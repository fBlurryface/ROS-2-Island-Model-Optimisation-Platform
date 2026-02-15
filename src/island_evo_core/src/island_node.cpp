#include "island_evo_core/island_node.hpp"
#include "island_evo_core/genetic_algorithm.hpp"
#include "island_evo_core/pso_algorithm.hpp"
#include "island_evo_core/de_algorithm.hpp"
#include <cmath>
#include <algorithm>
#include <cctype>

using std::placeholders::_1;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

IslandNode::IslandNode()
: Node("island_node"),
  state_(NodeState::INIT_CHECK),
  is_finished_(false)
{
  // 1. 参数声明
  this->declare_parameter("island_id", "Island_Alpha");
  this->declare_parameter("migration_interval", 20);
  this->declare_parameter("target_island", "ALL");
  this->declare_parameter("max_generations", 0);
  this->declare_parameter("timer_period_ms", 1);
  this->declare_parameter("required_senders_list", std::vector<std::string>({})); // 名单

  // 算法参数...
  this->declare_parameter("gene_dim", 30);
  this->declare_parameter("pop_size", 50);
  this->declare_parameter("function_name", "Rastrigin");
  this->declare_parameter("algorithm_type", "DE");

  // DE
  this->declare_parameter("de_F", 0.5);
  this->declare_parameter("de_CR", 0.9);

  // PSO
  this->declare_parameter("pso_w", 0.7);
  this->declare_parameter("pso_c1", 1.5);
  this->declare_parameter("pso_c2", 1.5);

  // GA
  this->declare_parameter("crossover_rate", 0.8);
  this->declare_parameter("mutation_rate", 0.1);

  // Objective-function params
  this->declare_parameter("rastrigin_A", 10.0);
  this->declare_parameter("michalewicz_m", 10);
  this->declare_parameter("lunacek_d", 1.0);
  this->declare_parameter("lunacek_s", 1.0);

  // 2. 参数读取
  island_id_ = this->get_parameter("island_id").as_string();
  migration_interval_ = this->get_parameter("migration_interval").as_int();
  target_island_ = this->get_parameter("target_island").as_string();
  max_generations_ = this->get_parameter("max_generations").as_int();

  // 构建名单 Set
  std::vector<std::string> list = this->get_parameter("required_senders_list").as_string_array();
  for (const auto& s : list) expected_senders_list_.insert(s);

  RCLCPP_INFO(this->get_logger(), "📋 ROLL CALL LIST: Waiting for %ld specific peer(s):", expected_senders_list_.size());
  for (const auto& s : expected_senders_list_) RCLCPP_INFO(this->get_logger(), "   - %s", s.c_str());

  // 3. 通信初始化 (必须 Reliable)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
  packet_pub_ = this->create_publisher<island_evo_core::msg::Packet>("migration_packets", qos);
  packet_sub_ = this->create_subscription<island_evo_core::msg::Packet>(
    "migration_packets", qos, std::bind(&IslandNode::migration_callback, this, _1));

  // 4. 算法
  init_algorithm();

  // 5. 启动状态机
  int timer_ms = this->get_parameter("timer_period_ms").as_int();
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_ms),
      std::bind(&IslandNode::state_machine_callback, this));
}

void IslandNode::state_machine_callback() {
    if (!algorithm_instance_ || is_finished_) return;

    // --- 状态 1: 启动检查 ---
    if (state_ == NodeState::INIT_CHECK) {
        if (!expected_senders_list_.empty()) {
            if (packet_pub_->get_subscription_count() == 0) return; // 等物理连接
        }
        RCLCPP_INFO(this->get_logger(), "✅ Network Connected. Engine START.");
        state_ = NodeState::COMPUTING;
        return;
    }

    // --- 状态 2: 计算 ---
    if (state_ == NodeState::COMPUTING) {
        int current_gen = algorithm_instance_->current_generation;

        // 结束检查
        if (max_generations_ > 0 && current_gen >= max_generations_) {
            auto best = algorithm_instance_->get_best_individual();
            RCLCPP_INFO(this->get_logger(), "🏁 FINISHED at Gen %d | Final: %.5e", current_gen, best.fitness);
            is_finished_ = true;
            return;
        }

        // 跑一批
        for (int k = 0; k < migration_interval_; ++k) algorithm_instance_->step();

        // 打印
        auto best = algorithm_instance_->get_best_individual();
        RCLCPP_INFO(this->get_logger(), "📊 Gen %d | Best: %.5e",
            algorithm_instance_->current_generation, best.fitness);

        // 发送带时间戳的包
        publish_migration();

        // 准备进入同步等待
        current_received_senders_.clear(); // 清空签到表
        state_ = NodeState::SYNC_WAIT;

        // 🔥 关键：进入等待的第一件事，去保险箱看看有没有“未来包”变“现在包”了
        check_future_buffer();

        // 也许 buffer 里已经齐了，直接检查一次
        check_sync_complete();
        return;
    }

    // --- 状态 3: 等待 ---
    if (state_ == NodeState::SYNC_WAIT) {
        check_sync_complete();
    }
}

void IslandNode::publish_migration() {
    auto msg = island_evo_core::msg::Packet();
    msg.source_id = island_id_;
    msg.target_id = target_island_;
    msg.generation = algorithm_instance_->current_generation; // 🏷️ 打标签

    auto best = algorithm_instance_->get_best_individual();
    auto ind_msg = island_evo_core::msg::Individual();
    ind_msg.genes = best.genes;
    ind_msg.fitness = best.fitness;
    msg.individuals.push_back(ind_msg);

    packet_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "📤 Sent Gen %d. Waiting for list...", msg.generation);
}

void IslandNode::migration_callback(const island_evo_core::msg::Packet::SharedPtr msg) {
    if (msg->source_id == island_id_) return;

    // 1. 名单鉴权
    if (expected_senders_list_.find(msg->source_id) == expected_senders_list_.end()) {
        return; // 不在名单上，滚
    }

    int my_gen = algorithm_instance_->current_generation;
    int pkt_gen = msg->generation;

    // 2. 代数判断
    if (pkt_gen < my_gen) {
        return;
    }
    else if (pkt_gen == my_gen) {
        process_packet(*msg);
    }
    else {
        future_buffer_[pkt_gen].push_back(*msg);
        RCLCPP_INFO(this->get_logger(), "📦 Buffered FUTURE Gen %d from %s", pkt_gen, msg->source_id.c_str());
    }
}

void IslandNode::process_packet(const island_evo_core::msg::Packet& pkt) {
    if (current_received_senders_.count(pkt.source_id)) return;

    if (algorithm_instance_) {
        for (const auto& ind : pkt.individuals) {
            algorithm_instance_->add_migrant(ind.genes, ind.fitness);
        }
    }

    current_received_senders_.insert(pkt.source_id);
    RCLCPP_INFO(this->get_logger(), "📦 Verified Recv: %s [%ld/%ld]",
        pkt.source_id.c_str(), current_received_senders_.size(), expected_senders_list_.size());

    check_sync_complete();
}

void IslandNode::check_future_buffer() {
    int my_gen = algorithm_instance_->current_generation;

    if (future_buffer_.count(my_gen)) {
        auto& packets = future_buffer_[my_gen];
        for (const auto& pkt : packets) {
            process_packet(pkt);
        }
        future_buffer_.erase(my_gen);
    }
}

void IslandNode::check_sync_complete() {
    if (state_ != NodeState::SYNC_WAIT) return;

    if (current_received_senders_.size() >= expected_senders_list_.size()) {
        int gen = algorithm_instance_->current_generation;
        RCLCPP_INFO(this->get_logger(), "⚡ Gen %d Sync Complete. Proceeding.", gen);
        state_ = NodeState::COMPUTING;
    }
}

void IslandNode::init_algorithm() {
    const int gene_dim = this->get_parameter("gene_dim").as_int();
    const int pop_size = this->get_parameter("pop_size").as_int();

    auto normalize = [](std::string s) {
        std::string out;
        out.reserve(s.size());
        for (unsigned char ch : s) {
            if (std::isalnum(ch)) out.push_back(static_cast<char>(std::tolower(ch)));
        }
        return out;
    };

    const std::string algo_raw = this->get_parameter("algorithm_type").as_string();
    const std::string func_raw = this->get_parameter("function_name").as_string();
    const std::string algo_key = normalize(algo_raw);
    const std::string func_key = normalize(func_raw);

    // -------------------------
    // Objective function factory
    // -------------------------
    OptimizationAlgorithm::FitnessFunction func;

    if (func_key == "rastrigin") {
        const double A = this->get_parameter("rastrigin_A").as_double();
        func = [A](const std::vector<double>& x) {
            double s = 0.0;
            for (double v : x) {
                s += (v * v - A * std::cos(2.0 * M_PI * v));
            }
            return A * static_cast<double>(x.size()) + s;
        };
    }
    else if (func_key == "michalewicz") {
        const int m = this->get_parameter("michalewicz_m").as_int();
        func = [m](const std::vector<double>& x) {
            // f(x) = - Σ sin(x_i) * [sin((i*x_i^2)/π)]^(2m)
            double sum = 0.0;
            for (size_t i = 0; i < x.size(); ++i) {
                const double xi = x[i];
                const double t1 = std::sin(xi);
                const double t2 = std::sin(((static_cast<double>(i) + 1.0) * xi * xi) / M_PI);
                sum += t1 * std::pow(t2, 2.0 * static_cast<double>(m));
            }
            return -sum;
        };
    }
    else if (func_key == "lunacek" || func_key == "lunacekbir" || func_key == "lunacekbirastrigin") {
        const double d = this->get_parameter("lunacek_d").as_double();
        const double s_param = this->get_parameter("lunacek_s").as_double();

        func = [d, s_param](const std::vector<double>& x) {
            // 常见简化 Lunacek bi-Rastrigin：
            // min( Σ(x-μ1)^2 , d*n + s*Σ(x-μ2)^2 ) + 10*Σ(1 - cos(2π(x-μ1)))
            const double mu1 = 2.5;
            const double mu2 = -2.5;

            double sum1 = 0.0, sum2 = 0.0, ras = 0.0;
            for (double v : x) {
                const double a = v - mu1;
                const double b = v - mu2;
                sum1 += a * a;
                sum2 += b * b;
                ras  += (1.0 - std::cos(2.0 * M_PI * a));
            }
            const double n = static_cast<double>(x.size());
            const double bowl = std::min(sum1, d * n + s_param * sum2);
            return bowl + 10.0 * ras;
        };
    }
    else if (func_key == "sphere" || func_key == "quadratic" || func_key == "sumxsq" || func_key.empty()) {
        func = [](const std::vector<double>& x) {
            double s = 0.0;
            for (double v : x) s += v * v;
            return s;
        };
    }
    else {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown function_name='%s' (normalized='%s'). Falling back to Sphere.",
                    func_raw.c_str(), func_key.c_str());
        func = [](const std::vector<double>& x) {
            double s = 0.0;
            for (double v : x) s += v * v;
            return s;
        };
    }

    // -------------------------
    // Algorithm factory (case-insensitive)
    // -------------------------
    if (algo_key == "de" || algo_key == "differentialevolution" || algo_key == "deall") {
        const double F  = this->get_parameter("de_F").as_double();
        const double CR = this->get_parameter("de_CR").as_double();
        algorithm_instance_ = std::make_unique<DEAlgorithm>(pop_size, gene_dim, F, CR, func);

        RCLCPP_INFO(this->get_logger(),
                    "🧬 Algorithm=DE | function=%s | pop=%d dim=%d | F=%.3f CR=%.3f",
                    func_raw.c_str(), pop_size, gene_dim, F, CR);
    }
    else if (algo_key == "pso" || algo_key == "particleswarm" || algo_key == "particleswarmoptimization" || algo_key == "psoall") {
        const double w  = this->get_parameter("pso_w").as_double();
        const double c1 = this->get_parameter("pso_c1").as_double();
        const double c2 = this->get_parameter("pso_c2").as_double();
        algorithm_instance_ = std::make_unique<PSOAlgorithm>(pop_size, gene_dim, w, c1, c2, func);

        RCLCPP_INFO(this->get_logger(),
                    "🧬 Algorithm=PSO | function=%s | pop=%d dim=%d | w=%.3f c1=%.3f c2=%.3f",
                    func_raw.c_str(), pop_size, gene_dim, w, c1, c2);
    }
    else if (algo_key == "ga" || algo_key == "genetic" || algo_key == "geneticalgorithm" || algo_key == "gaall") {
        const double cr = this->get_parameter("crossover_rate").as_double();
        const double mr = this->get_parameter("mutation_rate").as_double();
        algorithm_instance_ = std::make_unique<GeneticAlgorithm>(pop_size, gene_dim, cr, mr, func);

        RCLCPP_INFO(this->get_logger(),
                    "🧬 Algorithm=GA | function=%s | pop=%d dim=%d | crossover=%.3f mutation=%.3f",
                    func_raw.c_str(), pop_size, gene_dim, cr, mr);
    }
    else {
        const double cr = this->get_parameter("crossover_rate").as_double();
        const double mr = this->get_parameter("mutation_rate").as_double();

        RCLCPP_WARN(this->get_logger(),
                    "Unknown algorithm_type='%s' (normalized='%s'). Falling back to GA.",
                    algo_raw.c_str(), algo_key.c_str());

        algorithm_instance_ = std::make_unique<GeneticAlgorithm>(pop_size, gene_dim, cr, mr, func);

        RCLCPP_INFO(this->get_logger(),
                    "🧬 Algorithm=GA(fallback) | function=%s | pop=%d dim=%d | crossover=%.3f mutation=%.3f",
                    func_raw.c_str(), pop_size, gene_dim, cr, mr);
    }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IslandNode>());
  rclcpp::shutdown();
  return 0;
}

