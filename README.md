# island_evo_core (ROS 2)

Distributed **island-model evolutionary computation** implemented as a ROS 2 package.
Each island is a ROS 2 node running an optimizer (GA / PSO / DE). Islands periodically publish their best individual and synchronize with peers according to a chosen topology.

## What’s inside

- **C++ node**: `island_node` (runs one island)
- **Python monitor nodes** (optional):
  - `monitor_node.py` – plots fitness vs. migration events
  - `generation_monitor_node.py` – plots fitness vs. generations (uses `Packet.generation`)
- **Custom messages**:
  - `Individual.msg` – genes + fitness
  - `Packet.msg` – migration packet with `source_id`, `target_id`, `generation`, and individuals
- **Launch**: `island_system.launch.py`
  - Spawns N islands
  - Topology: `FULL` or `RING`
  - Algorithm allocation: `GA`, `PSO`, `DE` (all islands or per-island list)
  - Accepts a JSON string via `system_config`

## Quick start (Docker – recommended)

Build the image from the repository root:

```bash
docker build -t island-evo:latest .
```

Run an interactive shell:

```bash
docker run -it --rm --net=host island-evo:latest
```

Inside the container, you can launch the system:

```bash
ros2 launch island_evo_core island_system.launch.py
```

### Passing a config (JSON)

Example: 4 islands, ring topology, mixed algorithms, 200 generations:

```bash
ros2 launch island_evo_core island_system.launch.py \
  system_config:='{"num_islands":4,"topology":"RING","gene_dim":30,"target_func":"Rastrigin",\
  "algo_allocation":["GA","PSO","DE","GA"],\
  "common_params":{"max_generations":200,"timer_period_ms":1,"pop_size":50,"migration_interval":20}}'
```

## Local build (without Docker)

Requirements:
- ROS 2 Jazzy (or compatible)
- `rosdep`, `colcon`

From the workspace root:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Notes for publishing

Before making the repository public, consider updating:
- `src/island_evo_core/package.xml`: maintainer name/email, and license field

## License

This repository includes an Apache-2.0 license (see `LICENSE`).
