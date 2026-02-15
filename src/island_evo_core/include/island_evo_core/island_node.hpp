#ifndef ISLAND_EVO_CORE__ISLAND_NODE_HPP_
#define ISLAND_EVO_CORE__ISLAND_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <memory> 
#include <set>
#include <map>

#include "island_evo_core/msg/packet.hpp"
#include "island_evo_core/msg/individual.hpp"
#include "island_evo_core/optimization_algorithm.hpp"

// 严格状态机
enum class NodeState {
    INIT_CHECK,   // 启动前检查连接
    COMPUTING,    // 计算中 (不可打断)
    SYNC_WAIT     // 挂起，等待数据同步
};

class IslandNode : public rclcpp::Node
{
public:
  IslandNode();

private:
  // 核心循环
  void state_machine_callback(); 
  
  // 动作
  void publish_migration();
  void migration_callback(const island_evo_core::msg::Packet::SharedPtr msg);
  void init_algorithm(); 
  
  // 同步辅助函数
  void process_packet(const island_evo_core::msg::Packet& pkt); // 处理单个包
  void check_future_buffer();  // 检查是否有滞留的未来包
  void check_sync_complete();  // 检查是否齐了

  // 基础参数
  std::string island_id_;
  int migration_interval_;
  std::string target_island_;
  int max_generations_; 
  
  // 同步控制变量
  NodeState state_;
  bool is_finished_;
  
  // 1. 名单 (从 Launch 传入，不可变)
  std::set<std::string> expected_senders_list_; 
  
  // 2. 当期签到表 (记录当前代数已收到谁的包)
  std::set<std::string> current_received_senders_;

  // 3. 未来保险箱 (Key=代数, Value=包列表)
  // 用于暂存跑得快的节点发来的包，防止丢包
  std::map<int, std::vector<island_evo_core::msg::Packet>> future_buffer_;

  // ROS 组件
  rclcpp::Publisher<island_evo_core::msg::Packet>::SharedPtr packet_pub_;
  rclcpp::Subscription<island_evo_core::msg::Packet>::SharedPtr packet_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<OptimizationAlgorithm> algorithm_instance_;
};

#endif
