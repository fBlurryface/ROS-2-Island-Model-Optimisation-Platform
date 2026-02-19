import os
import json
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _merge_config(default_cfg: dict, user_cfg: dict) -> dict:
    """Merge user config into defaults. common_params is merged deeply."""
    cfg = dict(default_cfg)

    # Merge common_params first (deep merge only for this sub-dict)
    user_common = user_cfg.get("common_params", None)
    if isinstance(user_common, dict):
        cfg["common_params"] = dict(cfg.get("common_params", {}))
        cfg["common_params"].update(user_common)

    # Merge other top-level keys (except common_params)
    for k, v in user_cfg.items():
        if k == "common_params":
            continue
        cfg[k] = v

    return cfg


def _pick_algo_for_island(alloc, i: int) -> str:
    """
    alloc can be:
      - list: ["DE","PSO","GA"] -> per island
      - string:
          * "GA_ALL" -> "GA"
          * "DE_ALL" -> "DE" (etc.)
          * "GA"/"DE"/"PSO" -> that algo for all
    """
    # default
    algo = "GA"

    if isinstance(alloc, list) and len(alloc) > 0:
        # if list is shorter than num_islands, repeat last entry
        algo = alloc[i] if i < len(alloc) else alloc[-1]
    elif isinstance(alloc, str) and alloc.strip():
        s = alloc.strip().upper()
        if s.endswith("_ALL"):
            s = s[:-4]  # remove "_ALL"
        algo = s

    algo = str(algo).strip().upper()
    if algo not in {"GA", "DE", "PSO"}:
        # keep system running even if user typo
        print(f"⚠️ Unknown algorithm '{algo}' for island {i}. Falling back to GA.")
        algo = "GA"
    return algo


def launch_setup(context, *args, **kwargs):
    config_str = LaunchConfiguration("system_config").perform(context).strip()

    # ---------- defaults ----------
    default_config = {
        "num_islands": 4,
        "topology": "FULL",          # FULL / RING
        "gene_dim": 30,
        "target_func": "Rastrigin",  # forwarded as function_name
        "algo_allocation": "GA_ALL", # "GA_ALL" or list like ["DE","PSO","GA"]
        "enable_csv_log": False,     # allow user override
        "log_base_dir": None,        # optional override, default cwd/evo_logs
        "common_params": {
            "max_generations": 300,
            "timer_period_ms": 1,
            "pop_size": 50,
            "migration_interval": 10
        }
    }

    # ---------- parse user config ----------
    config = dict(default_config)
    if config_str:
        try:
            user_config = json.loads(config_str)
            if not isinstance(user_config, dict):
                raise ValueError("system_config JSON must be an object/dict.")
            config = _merge_config(default_config, user_config)
        except Exception as e:
            print(f"❌ Failed to parse system_config JSON. Using defaults.\n   Error: {e}\n   Raw: {config_str}")

    # ---------- validate / normalize ----------
    try:
        num_islands = int(config.get("num_islands", default_config["num_islands"]))
    except Exception:
        num_islands = int(default_config["num_islands"])
    if num_islands < 1:
        print("⚠️ num_islands < 1. Forcing to 1.")
        num_islands = 1

    topo = str(config.get("topology", "FULL")).strip().upper()
    if topo not in {"FULL", "RING"}:
        print(f"⚠️ Unknown topology '{topo}'. Falling back to FULL.")
        topo = "FULL"

    gene_dim = int(config.get("gene_dim", default_config["gene_dim"]))
    target_func = str(config.get("target_func", default_config["target_func"]))

    enable_csv_log = bool(config.get("enable_csv_log", default_config["enable_csv_log"]))

    # ---------- log directory ----------
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_base_dir = config.get("log_base_dir", None)
    if not log_base_dir:
        log_base_dir = os.path.join(os.getcwd(), "evo_logs")
    run_log_dir = os.path.join(log_base_dir, f"run_{timestamp}")
    os.makedirs(run_log_dir, exist_ok=True)
    print(f"📁 Logs will be saved to: {run_log_dir}")

    # ---------- build nodes ----------
    nodes = []
    alloc = config.get("algo_allocation", default_config["algo_allocation"])

    # Helpful debug summary
    print(f"🧩 Islands: {num_islands} | Topology: {topo} | Func: {target_func} | Dim: {gene_dim} | CSV: {enable_csv_log}")
    print(f"🧠 algo_allocation: {alloc}")

    for i in range(num_islands):
        island_id = f"Island_{i}"

        # Who do we wait for each generation? (barrier sync dependency)
        required_list = []
        if num_islands > 1:
            if topo == "FULL":
                required_list = [f"Island_{j}" for j in range(num_islands) if j != i]
            elif topo == "RING":
                required_list = [f"Island_{(i - 1 + num_islands) % num_islands}"]

        algo_i = _pick_algo_for_island(alloc, i)
        print(f"  - {island_id}: algorithm_type={algo_i}, required_senders_list={required_list}")

        node_params = {
            "island_id": island_id,
            "log_dir": run_log_dir,
            "enable_csv_log": enable_csv_log,
            "function_name": target_func,
            "gene_dim": gene_dim,
            "required_senders_list": required_list,

            # ✅ This is the missing piece in your original file:
            "algorithm_type": algo_i,
        }

        # Merge common runtime params
        common_params = config.get("common_params", {})
        if isinstance(common_params, dict):
            node_params.update(common_params)

        nodes.append(
            Node(
                package="island_evo_core",
                executable="island_node",
                name=f"island_{i}",
                parameters=[node_params],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "system_config",
            default_value="",
            description="JSON string config for island system (num_islands, topology, algo_allocation, target_func, common_params, etc.)"
        ),
        OpaqueFunction(function=launch_setup),
    ])
