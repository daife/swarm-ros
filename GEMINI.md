# swarm-ros

## Project Overview

`swarm-ros` is a ROS 2-based multi-UAV pursuit-evasion game simulation and training system using Gazebo Harmonic and deep reinforcement learning. The system implements a multi-agent environment where pursuer drones chase evader drones in obstacle-filled environments.

### Main Technologies
- **ROS 2 Jazzy**
- **Gazebo Harmonic**
- **Deep Reinforcement Learning (PyTorch)**
- **Python 3.12**

### Core Architecture
The system consists of several specialized ROS 2 packages interacting through topics and services:
1.  **`world_manager`**: Maintains obstacle topologies (random or fixed layouts), staging areas, and broadcasts them.
2.  **`agent_manager`**: Performs collision-aware agent spawning/sampling and resets drones safely using Gazebo services.
3.  **`rl_policy`**: Runs the Reinforcement Learning policy algorithms (MADDPG for pursuers, DDPG for evaders).
4.  **`training_coordinator`**: Orchestrates episode scheduling, reset coordination, and done condition evaluation (capture/collision/timeout).
5.  **`sim_bridge`**: Bridges Gazebo topics to ROS messages and translates ROS acceleration commands into Gazebo entity movements (via continuous velocity integration).
6.  **`swarm_bringup`**: Central launch configuration, managing Gazebo world and model assets, and unifying parameters across components.

## Building and Running

### Environment Setup

The project uses a Python virtual environment and requires specific ROS 2/Gazebo configurations.

```bash
# Activate Python virtual environment and ROS 2 setup
source env.sh

# Build the workspace using colcon
colcon build

# Source the workspace
source install/setup.bash
```

### Running the System

**One-click launch (recommended):**
```bash
# Run with default configuration
./bash/30_training_launch.sh

# Run with specific predefined configuration profiles
./bash/30_training_launch.sh bash/configs/train_from_scratch.env.sh
./bash/30_training_launch.sh bash/configs/eval_only.env.sh
```

**Manual launch (for debugging):**
You can launch nodes individually using the scripts in the `bash/` directory (e.g., `./bash/10_gazebo_gui.sh`, `./bash/11_parameter_bridge.sh`, `./bash/20_world_manager_node.sh`, etc.).

## Development Conventions

-   **ROS 2 Python Nodes**: The packages are built using `ament_python`. Entry points are defined in `setup.py` of each package.
-   **Configuration Hierarchy**: The system uses hierarchical configuration:
    1.  Launch arguments in `training_launch.py` (highest priority).
    2.  Environment variables in `bash/env_defaults.sh` and `bash/configs/*.env.sh`.
    3.  Node parameters with default fallback values (lowest priority).
-   **Reset Synchronization**: The project employs a strict three-way handshake (`world_manager`, `agent_manager`, `rl_policy`) coordinated by `training_coordinator` using `reset_id` to ensure synchronized episode starts without getting stuck.
-   **SI Units**: Standard SI units are used across all parameters (meters, m/s, m/s², seconds).
