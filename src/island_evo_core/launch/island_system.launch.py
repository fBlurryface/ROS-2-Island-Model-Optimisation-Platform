import os
import json
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    config_str = LaunchConfiguration('system_config').perform(context)
    
    # 默认配置
    config = {
        "num_islands": 4,
        "topology": "FULL",
        "gene_dim": 30,
        "target_func": "Rastrigin",
        "algo_allocation": "GA_ALL",
        "common_params": {
            "max_generations": 300,
            "timer_period_ms": 1,
            "pop_size": 50,
            "migration_interval": 10
        }
    }

    try:
        if config_str:
            user_config = json.loads(config_str)
            if "common_params" in user_config:
                config["common_params"].update(user_config["common_params"])
                del user_config["common_params"]
            config.update(user_config)
    except Exception: pass

    # 🔥 自动生成日志文件夹路径
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_log_dir = os.path.join(os.getcwd(), "evo_logs", f"run_{timestamp}")
    print(f"📁 Logs will be saved to: {run_log_dir}")

    nodes = []
    num_islands = int(config["num_islands"])
    topo = str(config["topology"]).upper()
    
    for i in range(num_islands):
        island_id = f"Island_{i}"
        required_list = []
        if num_islands > 1:
            if topo == "FULL":
                required_list = [f"Island_{j}" for j in range(num_islands) if i != j]
            elif topo == "RING":
                required_list = [f"Island_{(i-1+num_islands)%num_islands}"]

        node_params = {
            "island_id": island_id,
            "log_dir": run_log_dir,  # 传递给节点
            "enable_csv_log": False,
            "function_name": config["target_func"],
            "gene_dim": int(config["gene_dim"]),
            "required_senders_list": required_list
        }
        node_params.update(config["common_params"])

        nodes.append(Node(
            package="island_evo_core",
            executable="island_node",
            name=f"island_{i}",
            parameters=[node_params]
        ))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("system_config", default_value=""),
        OpaqueFunction(function=launch_setup)
    ])