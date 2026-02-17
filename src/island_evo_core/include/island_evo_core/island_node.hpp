#ifndef ISLAND_EVO_CORE__ISLAND_NODE_HPP_
#define ISLAND_EVO_CORE__ISLAND_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <memory> 
#include <set>
#include <map>
#include <fstream> 

#include "island_evo_core/msg/packet.hpp"
#include "island_evo_core/msg/individual.hpp"
#include "island_evo_core/optimization_algorithm.hpp"

enum class NodeState { INIT_CHECK, COMPUTING, SYNC_WAIT };

class IslandNode : public rclcpp::Node
{
public:
  IslandNode();
  ~IslandNode(); 

private:
  void state_machine_callback(); 
  void publish_migration(); 
  void record_local_log(); 
  
  void migration_callback(const island_evo_core::msg::Packet::SharedPtr msg);
  void init_algorithm(); 
  void process_packet(const island_evo_core::msg::Packet& pkt);
  void check_future_buffer();
  void check_sync_complete();

  std::string island_id_;
  int migration_interval_;
  std::string target_island_;
  int max_generations_; 
  bool enable_csv_log_;   
  std::string log_dir_;   
  
  NodeState state_;
  bool is_finished_;
  
  std::set<std::string> expected_senders_list_; 
  std::set<std::string> current_received_senders_;
  std::map<int, std::vector<island_evo_core::msg::Packet>> future_buffer_;

  rclcpp::Publisher<island_evo_core::msg::Packet>::SharedPtr packet_pub_;
  rclcpp::Subscription<island_evo_core::msg::Packet>::SharedPtr packet_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream csv_out_; 

  std::unique_ptr<OptimizationAlgorithm> algorithm_instance_;
};

#endif