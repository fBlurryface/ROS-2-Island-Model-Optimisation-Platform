#include "island_evo_core/island_node.hpp"
#include "island_evo_core/genetic_algorithm.hpp"
#include "island_evo_core/pso_algorithm.hpp"
#include "island_evo_core/de_algorithm.hpp"
#include <cmath>
#include <algorithm>
#include <cctype>
#include <filesystem> // C++17 目录操作

using std::placeholders::_1;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

IslandNode::IslandNode()
: Node("island_node"), state_(NodeState::INIT_CHECK), is_finished_(false)
{
  // 1. 参数声明 (必须完整，防止 ParameterNotDeclaredException)
  this->declare_parameter("island_id", "Island_Alpha");
  this->declare_parameter("migration_interval", 20);
  this->declare_parameter("target_island", "ALL");
  this->declare_parameter("max_generations", 300);
  this->declare_parameter("timer_period_ms", 10);
  this->declare_parameter("enable_csv_log", false);
  this->declare_parameter("log_dir", "./evo_logs");
  this->declare_parameter("required_senders_list", std::vector<std::string>({}));
  this->declare_parameter("gene_dim", 30);
  this->declare_parameter("pop_size", 50);
  this->declare_parameter("function_name", "Rastrigin");
  this->declare_parameter("algorithm_type", "GA");

  // DE 参数
  this->declare_parameter("de_F", 0.5);
  this->declare_parameter("de_CR", 0.9);
  // PSO 参数
  this->declare_parameter("pso_w", 0.7);
  this->declare_parameter("pso_c1", 1.5);
  this->declare_parameter("pso_c2", 1.5);
  // GA 参数
  this->declare_parameter("crossover_rate", 0.8);
  this->declare_parameter("mutation_rate", 0.1);
  // 函数参数
  this->declare_parameter("rastrigin_A", 10.0);
  this->declare_parameter("michalewicz_m", 10);
  this->declare_parameter("lunacek_d", 1.0);
  this->declare_parameter("lunacek_s", 1.0);

  // 2. 参数读取
  island_id_ = this->get_parameter("island_id").as_string();
  migration_interval_ = this->get_parameter("migration_interval").as_int();
  target_island_ = this->get_parameter("target_island").as_string();
  max_generations_ = this->get_parameter("max_generations").as_int();
  enable_csv_log_ = this->get_parameter("enable_csv_log").as_bool();
  log_dir_ = this->get_parameter("log_dir").as_string();

  std::vector<std::string> list = this->get_parameter("required_senders_list").as_string_array();
  for (const auto& s : list) expected_senders_list_.insert(s);

  // 3. 本地日志初始化 (分布式写文件)
  if (enable_csv_log_) {
      std::filesystem::create_directories(log_dir_);
      std::string file_path = log_dir_ + "/" + island_id_ + ".csv";
      csv_out_.open(file_path, std::ios::out);
      if (csv_out_.is_open()) {
          csv_out_ << "Generation,Best_Fitness\n";
      }
  }

  // 4. 通信初始化 (仅保留迁移通道，减轻网络压力)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
  packet_pub_ = this->create_publisher<island_evo_core::msg::Packet>("migration_packets", qos);
  packet_sub_ = this->create_subscription<island_evo_core::msg::Packet>(
    "migration_packets", qos, std::bind(&IslandNode::migration_callback, this, _1));

  init_algorithm();

  int timer_ms = this->get_parameter("timer_period_ms").as_int();
  timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&IslandNode::state_machine_callback, this));
}

IslandNode::~IslandNode() {
    if (csv_out_.is_open()) csv_out_.close();
}

void IslandNode::state_machine_callback() {
    if (!algorithm_instance_ || is_finished_) return;

    if (state_ == NodeState::INIT_CHECK) {
        if (!expected_senders_list_.empty() && packet_pub_->get_subscription_count() == 0) return;
        state_ = NodeState::COMPUTING;
        return;
    }

    if (state_ == NodeState::COMPUTING) {
        if (max_generations_ > 0 && algorithm_instance_->current_generation >= max_generations_) {
            is_finished_ = true;
            if (csv_out_.is_open()) {
                // 这一步会强制把最后一段数据（比如最后 50 行）刷入硬盘，然后安全关闭
                csv_out_.close(); 
            }
            RCLCPP_INFO(this->get_logger(),
            "🏁 %s Reached max_generations=%d. Finished.",
            island_id_.c_str(),
            max_generations_);
            return;
        }

        for (int k = 0; k < migration_interval_; ++k) {
            algorithm_instance_->step();
            record_local_log(); // 本地写磁盘
        }

        publish_migration();
        current_received_senders_.clear();
        state_ = NodeState::SYNC_WAIT;
        check_future_buffer();
        check_sync_complete();
        return;
    }

    if (state_ == NodeState::SYNC_WAIT) check_sync_complete();
}

void IslandNode::record_local_log() {
    if (enable_csv_log_ && csv_out_.is_open()) {
        auto best = algorithm_instance_->get_best_individual();
        csv_out_ << algorithm_instance_->current_generation << "," << best.fitness << "\n";
    }
}

void IslandNode::publish_migration() {
    auto msg = island_evo_core::msg::Packet();
    msg.source_id = island_id_;
    msg.target_id = target_island_;
    msg.generation = algorithm_instance_->current_generation;
    auto best = algorithm_instance_->get_best_individual();
    auto ind_msg = island_evo_core::msg::Individual();
    ind_msg.genes = best.genes;
    ind_msg.fitness = best.fitness;
    msg.individuals.push_back(ind_msg);
    packet_pub_->publish(msg);
}

void IslandNode::migration_callback(const island_evo_core::msg::Packet::SharedPtr msg) {
    if (msg->source_id == island_id_) return;
    if (expected_senders_list_.find(msg->source_id) == expected_senders_list_.end()) return;
    int pkt_gen = msg->generation;
    int my_gen = algorithm_instance_->current_generation;
    if (pkt_gen < my_gen) return;
    if (pkt_gen == my_gen) process_packet(*msg);
    else future_buffer_[pkt_gen].push_back(*msg);
}

void IslandNode::process_packet(const island_evo_core::msg::Packet& pkt) {
    if (current_received_senders_.count(pkt.source_id)) return;
    if (algorithm_instance_) {
        for (const auto& ind : pkt.individuals) {
            algorithm_instance_->add_migrant(ind.genes, ind.fitness);
        }
    }
    current_received_senders_.insert(pkt.source_id);
    check_sync_complete();
}

void IslandNode::check_future_buffer() {
    int my_gen = algorithm_instance_->current_generation;
    if (future_buffer_.count(my_gen)) {
        for (const auto& pkt : future_buffer_[my_gen]) process_packet(pkt);
        future_buffer_.erase(my_gen);
    }
}

void IslandNode::check_sync_complete() {
    if (state_ != NodeState::SYNC_WAIT) return;
    if (current_received_senders_.size() >= expected_senders_list_.size()) state_ = NodeState::COMPUTING;
}

void IslandNode::init_algorithm() {
    const int gene_dim = this->get_parameter("gene_dim").as_int();
    const int pop_size = this->get_parameter("pop_size").as_int();
    auto normalize = [](std::string s) {
        std::string out;
        for (unsigned char ch : s) if (std::isalnum(ch)) out.push_back(static_cast<char>(std::tolower(ch)));
        return out;
    };
    const std::string algo_key = normalize(this->get_parameter("algorithm_type").as_string());
    const std::string func_key = normalize(this->get_parameter("function_name").as_string());
    OptimizationAlgorithm::FitnessFunction func;

    if (func_key == "rastrigin") {
        const double A = this->get_parameter("rastrigin_A").as_double();
        func = [A](const std::vector<double>& x) {
            double s = 0.0;
            for (double v : x) s += (v * v - A * std::cos(2.0 * M_PI * v));
            return A * static_cast<double>(x.size()) + s;
        };
    } else if (func_key == "michalewicz") {
        const int m = this->get_parameter("michalewicz_m").as_int();
        func = [m](const std::vector<double>& x) {
            double sum = 0.0;
            for (size_t i = 0; i < x.size(); ++i) {
                double xi = x[i];
                sum += std::sin(xi) * std::pow(std::sin(((i+1.0)*xi*xi)/M_PI), 2.0*m);
            }
            return -sum;
        };
    } else {
        func = [](const std::vector<double>& x) { double s = 0.0; for (double v : x) s += v * v; return s; };
    }

    if (algo_key == "de") {
        algorithm_instance_ = std::make_unique<DEAlgorithm>(pop_size, gene_dim, 
            this->get_parameter("de_F").as_double(), this->get_parameter("de_CR").as_double(), func);
    } else if (algo_key == "pso") {
        algorithm_instance_ = std::make_unique<PSOAlgorithm>(pop_size, gene_dim, 
            this->get_parameter("pso_w").as_double(), this->get_parameter("pso_c1").as_double(), 
            this->get_parameter("pso_c2").as_double(), func);
    } else {
        algorithm_instance_ = std::make_unique<GeneticAlgorithm>(pop_size, gene_dim, 
            this->get_parameter("crossover_rate").as_double(), this->get_parameter("mutation_rate").as_double(), func);
    }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IslandNode>());
  rclcpp::shutdown();
  return 0;
}