import os
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _norm_str(x: str) -> str:
    return str(x).strip()


def launch_setup(context, *args, **kwargs):
    config_str = LaunchConfiguration('system_config').perform(context)

    # 默认配置
    default_config = {
        "num_islands": 4,
        "topology": "FULL",
        "gene_dim": 30,
        "target_func": "Rastrigin",
        "algo_allocation": "GA_ALL",
        "common_params": {
            "max_generations": 1000,
            "timer_period_ms": 1,
            "pop_size": 50,
            "migration_interval": 20
        }
    }

    # 解析 JSON（顶层覆盖；common_params 做一层 merge）
    try:
        if config_str:
            print(f"📥 Loading external config: {config_str}")
            user_config = json.loads(config_str)

            if "common_params" in user_config:
                default_config["common_params"].update(user_config["common_params"])
                del user_config["common_params"]

            default_config.update(user_config)
    except json.JSONDecodeError as e:
        print(f"⚠️ Error parsing JSON config: {e}. Using defaults.")

    config = default_config
    nodes = []

    # 统一 topology 大小写
    config["topology"] = _norm_str(config.get("topology", "FULL")).upper()

    num_islands = int(config["num_islands"])
    algo_list = config["algo_allocation"]

    # ---------------------------------------------------------
    # 算法分配：支持字符串 / 列表，且大小写不敏感
    # ---------------------------------------------------------
    if isinstance(algo_list, str):
        key = _norm_str(algo_list).upper()
        if key in ("GA_ALL", "GA"):
            algo_list = ["GA"] * num_islands
        elif key in ("PSO_ALL", "PSO"):
            algo_list = ["PSO"] * num_islands
        elif key in ("DE_ALL", "DE"):
            algo_list = ["DE"] * num_islands
        else:
            # 未知 -> 全 GA
            algo_list = ["GA"] * num_islands
    else:
        # list / tuple：逐项归一化
        try:
            algo_list = [_norm_str(x).upper() for x in algo_list]
        except Exception:
            algo_list = ["GA"]

        # 空列表 -> 全 GA
        if len(algo_list) == 0:
            algo_list = ["GA"]

        # 过滤未知值（避免节点端 fallback 到 GA 而你以为是 DE）
        algo_list = [a if a in ("GA", "PSO", "DE") else "GA" for a in algo_list]

    # 复制填满到 num_islands（保持原有“重复序列”行为）
    while len(algo_list) < num_islands:
        algo_list.extend(algo_list)
    algo_list = algo_list[:num_islands]

    print(f"🚀 System Launching: {num_islands} Islands | Topology: {config['topology']}")

    # ---------------------------------------------------------
    # 🌐 核心：生成节点和名单
    # ---------------------------------------------------------
    for i in range(num_islands):
        node_name = f"island_{i}"
        island_id = f"Island_{i}"

        # 1) target_id：底层用广播，接收端靠 required_senders_list 过滤
        target_id = "ALL"

        # 2) 生成接收名单 (Roll Call List)
        required_list = []

        if num_islands > 1:
            if config["topology"] == "FULL":
                # 全连接：名单 = 除了我之外的所有人
                for j in range(num_islands):
                    if i != j:
                        required_list.append(f"Island_{j}")

            elif config["topology"] == "RING":
                # 单向环形：名单 = 只有我的上家 (Predecessor)
                # 逻辑：I_0 听 I_{N-1}, I_1 听 I_0 ...
                prev_index = (i - 1 + num_islands) % num_islands
                required_list.append(f"Island_{prev_index}")

            else:
                # 未知拓扑：回退 FULL（并提示）
                print(f"⚠️ Unknown topology '{config['topology']}', fallback to FULL.")
                for j in range(num_islands):
                    if i != j:
                        required_list.append(f"Island_{j}")

        # 3) 组装参数
        node_params = {
            "island_id": island_id,
            "target_island": target_id,
            "algorithm_type": algo_list[i],
            "function_name": config["target_func"],
            "gene_dim": int(config["gene_dim"]),
            "required_senders_list": required_list
        }

        # 合并 problem_params / common_params
        if "problem_params" in config and isinstance(config["problem_params"], dict):
            node_params.update(config["problem_params"])

        if "common_params" in config and isinstance(config["common_params"], dict):
            node_params.update(config["common_params"])

        node = Node(
            package="island_evo_core",
            executable="island_node",
            name=node_name,
            output="screen",
            emulate_tty=True,
            parameters=[node_params]
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "system_config",
        default_value="",
        description="JSON string config"
    )
    return LaunchDescription([config_arg, OpaqueFunction(function=launch_setup)])

