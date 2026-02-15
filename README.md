# ROS 2 Island-Model Optimisation Platform

![docker-build](https://github.com/fBlurryface/ROS-2-Island-Model-Optimisation-Platform/actions/workflows/docker-build.yml/badge.svg)

A synchronous island-model optimisation platform implemented in ROS 2 (Jazzy).
It scales to N islands with configurable topology (FULL / RING) and per-island algorithm selection (GA / PSO / DE).
Islands periodically exchange their best individuals and synchronise at a migration barrier.

## Highlights

- Scales to N islands (each island is a ROS 2 node)
- Topology
  - FULL: each island synchronises with all other islands
  - RING: each island synchronises with its predecessor only (unidirectional ring)
- Per-island algorithm allocation: mix GA / PSO / DE across islands
- Synchronous migration
  - migration packets are tagged with generation
  - islands wait until all required peers for the current generation have checked in
  - out-of-order packets are buffered and applied when they become current
- Optional real-time monitors (matplotlib)

## Repository layout

This repo is a ROS 2 workspace:

    ros2_docker_ws/
      Dockerfile
      docker-compose.yml
      src/
        island_evo_core/
          launch/
          msg/
          include/
          src/

The ROS 2 package is: island_evo_core

## What’s inside (package: island_evo_core)

### Nodes

- C++
  - island_node — runs one optimisation island (GA / PSO / DE)
- Python (optional monitors)
  - monitor_node.py — plots best fitness vs migration events
  - generation_monitor_node.py — plots best fitness vs generation (Packet.generation)

### Topics & messages

- Topic: migration_packets
- Message: Packet.msg
  - source_id, target_id, generation, Individual[]
- Message: Individual.msg
  - genes[], fitness

### Launch

- island_system.launch.py
  - spawns N islands
  - builds each island’s “roll-call list” (required_senders_list) based on topology
  - accepts a JSON string via system_config

## Quick start (Docker — recommended)

### Build

From repository root:

    docker build -t island-evo:latest .

### Run (interactive)

Note: --net=host is important so DDS discovery works smoothly in-container.

    docker run -it --rm --net=host island-evo:latest

Inside the container:

    ros2 launch island_evo_core island_system.launch.py

### Run with docker-compose

    docker compose up --build

## Running monitors (optional)

Open another terminal (same environment where ROS 2 workspace is sourced):

    ros2 run island_evo_core generation_monitor_node.py

or:

    ros2 run island_evo_core monitor_node.py

If you are running GUI plotting from Docker on Linux, you may need X11 permissions (e.g. xhost +local:).
On Windows, the simplest path is usually to run monitors on the host (or within WSL with an X server).

## Configuration (system_config JSON)

The launch file takes a JSON string via:

    ros2 launch island_evo_core island_system.launch.py system_config:='<JSON>'

### Default config (used when system_config is empty)

    {
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

### Top-level fields

- num_islands (int) — number of islands
- topology (string) — FULL or RING (case-insensitive)
- gene_dim (int) — dimension of the decision vector
- target_func (string) — objective function name (see below)
- algo_allocation (string or list)
  - string: GA_ALL / PSO_ALL / DE_ALL (also accepts GA, PSO, DE)
  - list: ["GA","PSO","DE","GA"]
  - if list is shorter than num_islands, it repeats to fill
- common_params (object) — merged into each island node’s parameters
- problem_params (object) — extra parameters merged into each island node’s parameters (see Algorithms & parameters)

### Example: 4 islands, ring topology, mixed algorithms, 200 generations

Bash / Linux / Docker:

    ros2 launch island_evo_core island_system.launch.py \
      system_config:='{"num_islands":4,"topology":"RING","gene_dim":30,"target_func":"Rastrigin","algo_allocation":["GA","PSO","DE","GA"],"common_params":{"max_generations":200,"timer_period_ms":1,"pop_size":50,"migration_interval":20}}'

PowerShell (recommended quoting):

    ros2 launch island_evo_core island_system.launch.py `
      system_config:='{"num_islands":4,"topology":"RING","gene_dim":30,"target_func":"Rastrigin","algo_allocation":["GA","PSO","DE","GA"],"common_params":{"max_generations":200,"timer_period_ms":1,"pop_size":50,"migration_interval":20}}'

## Supported objective functions

Set via target_func (case-insensitive; non-alphanumeric chars are ignored internally):

- Rastrigin (param: rastrigin_A, default 10.0)
- Michalewicz (param: michalewicz_m, default 10)
- Lunacek / LunacekBiRastrigin (params: lunacek_d, lunacek_s, defaults 1.0)
- Sphere (fallback if unknown)

## Algorithms & parameters

Per island, the node parameter algorithm_type selects the optimiser:

- DE
  - de_F (default 0.5)
  - de_CR (default 0.9)
- PSO
  - pso_w (default 0.7)
  - pso_c1 (default 1.5)
  - pso_c2 (default 1.5)
- GA
  - crossover_rate (default 0.8)
  - mutation_rate (default 0.1)

These can be injected through problem_params and/or common_params (they are merged into node parameters at launch time).

## Local build (without Docker)

Requirements:
- ROS 2 Jazzy (or compatible)
- rosdep, colcon

From the workspace root:

    rosdep install --from-paths src --ignore-src -y
    colcon build --symlink-install
    source install/setup.bash

Run:

    ros2 launch island_evo_core island_system.launch.py

## Troubleshooting

- DDS discovery / multi-node networking in Docker
  - use --net=host (already in examples / compose)
- No plots shown / matplotlib issues
  - run monitor nodes on host
  - or enable X11 forwarding properly (Linux)
- Topology behaviour
  - FULL: each island waits for all others per sync step
  - RING: each island waits for predecessor only

## License

Apache-2.0 — see LICENSE.

## Citation (optional)

If you want people to cite this repository:

    @software{ros2_island_model_platform,
      title        = {ROS 2 Island-Model Optimisation Platform},
      author       = {Zhu, Weizhe},
      year         = {2026},
      url          = {https://github.com/fBlurryface/ROS-2-Island-Model-Optimisation-Platform}
    }
