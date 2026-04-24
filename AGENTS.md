# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Repository Overview

This is a ROS 2-based multi-UAV pursuit-evasion game simulation and training system using Gazebo Harmonic and deep reinforcement learning. The system implements a multi-agent environment where pursuer drones chase evader drones in obstacle-filled environments.

## Key Commands and Development Workflow

### Environment Setup
```bash
# Activate Python virtual environment and ROS 2
source env.sh

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Running the System

**Option 1: One-click launch (recommended)**
```bash
# Run with default configuration  
./bash/30_training_launch.sh

# Run with specific configuration
./bash/30_training_launch.sh bash/configs/train_from_scratch.env.sh
./bash/30_training_launch.sh bash/configs/eval_only.env.sh
```

**Option 2: Manual multi-terminal launch**
```bash
# Terminal 1: Gazebo GUI
./bash/10_gazebo_gui.sh

# Terminal 2: Parameter bridge
./bash/11_parameter_bridge.sh

# Terminal 3: World manager
./bash/20_world_manager_node.sh

# Terminal 4: Agent manager
./bash/21_agent_manager_node.sh

# Terminal 5: Simulation bridge
./bash/22_sim_bridge_node.sh

# Terminal 6: RL policy
./bash/23_rl_policy_node.sh

# Terminal 7: Training coordinator
./bash/24_training_coordinator_node.sh
```

### Manual Control Commands
```bash
# Reset world obstacles
ros2 topic pub --once /scheduler/reset_world std_msgs/msg/Int32 "{data: 1}"

# Reset agent positions (mode=1: fixed, mode=2: random)
ros2 topic pub --once /scheduler/reset_agents_cmd std_msgs/msg/Int32MultiArray "{data: [1, 2]}"

# Check reset poses
ros2 topic echo --once /agent_manager/reset_poses
```

### Parameter Configuration

Key configuration files:
- `bash/env_defaults.sh` - Default parameter values
- `bash/configs/*.env.sh` - Predefined configuration profiles
- `src/swarm_bringup/launch/training_launch.py` - Launch configuration

Common parameters to adjust:
- `NUM_PURSUERS` / `NUM_EVADERS` - Number of agents
- `MAX_EPISODES` / `MAX_STEPS` - Training duration
- `PURSUER_TRAIN_MODE` / `EVADER_TRAIN_MODE` - Training modes
- `OBSTACLE_DENSITY` - Environment difficulty
- `TORCH_DEVICE` - Training device (auto/cpu/cuda/cuda:0)

## Architecture Overview

### Core Components

1. **world_manager** - Manages obstacle generation and world state
   - Generates random/fixed obstacle layouts
   - Maintains obstacle pools and staging areas
   - Broadcasts obstacle positions via `/world_manager/obstacles`

2. **agent_manager** - Handles agent spawning and safe reset
   - Performs collision-aware agent positioning
   - Manages agent lifecycle in Gazebo
   - Publishes reset poses via `/agent_manager/reset_poses`

3. **sim_bridge** - Gazebo-ROS interface and motion control
   - Bridges Gazebo topics to ROS messages
   - Implements acceleration-based motion control
   - Filters ground contacts from collision detection

4. **rl_policy** - Reinforcement learning implementation
   - MADDPG for pursuers (multi-agent with centralized critic)
   - DDPG for evaders (single-agent)
   - Supports training modes: train_scratch/train_load/eval_only
   - Checkpoint management for model weights

5. **training_coordinator** - Training orchestration and metrics
   - Episode scheduling and reset coordination
   - Done condition evaluation (capture/collision/timeout)
   - Training metrics publishing via `/rl/episode_metrics`

6. **swarm_bringup** - Launch and asset management
   - Central launch configuration
   - Gazebo world and model assets
   - Parameter unification across components

### Data Flow Architecture

**Main Control Loop:**
1. Gazebo → SimBridgeNode → ROS odometry/collision topics
2. ROS topics → RLPolicyNode → Policy inference/training
3. RLPolicyNode → ROS acceleration commands → SimBridgeNode
4. SimBridgeNode → Gazebo set_pose service → Entity movement

**Reset Synchronization:**
1. TrainingCoordinator → Reset commands with reset_id
2. WorldManager + AgentManager + RLPolicy → Reset completion signals
3. Three-way handshake ensures synchronized episode starts

### Key ROS Topics

- `/agent_i/odom` - Agent odometry (nav_msgs/Odometry)
- `/agent_i/collision` - Collision status (std_msgs/Bool)
- `/agent_i/cmd_accel` - Acceleration commands (geometry_msgs/Accel)
- `/world_manager/obstacles` - Obstacle positions (visualization_msgs/MarkerArray)
- `/rl/episode_metrics` - Training metrics (std_msgs/Float32MultiArray)
- `/scheduler/reset_world` - World reset triggers
- `/scheduler/reset_agents_cmd` - Agent reset commands

### Training Metrics Format

The `/rl/episode_metrics` topic provides:
- Index 0: episode number
- Index 1: step count at episode end
- Index 2: done reason (1=capture, 2=collision, 3=timeout)
- Index 3: capture streak
- Index 4: collision events count
- Index 5: total collision penalty

## Development Guidelines

### Adding New Features

1. **New ROS Nodes**: Follow the existing pattern with setup.py entry_points
2. **Parameters**: Add to env_defaults.sh and training_launch.py
3. **Launch Config**: Update swarm_bringup/launch/training_launch.py
4. **Testing**: Use manual topic publishing for isolated component testing

### Common Modification Patterns

**Adding a new algorithm:**
1. Implement the algorithm class in rl_policy package
2. Add algorithm selection parameter
3. Update RLPolicyNode to support the new algorithm
4. Ensure input/output contract compatibility

**Modifying world generation:**
1. Update world_manager node logic
2. Adjust obstacle pool configurations
3. Update staging area coordinates if needed
4. Test with different seed modes

**Changing agent behavior:**
1. Modify sim_bridge control parameters
2. Adjust collision detection thresholds
3. Update agent_manager reset logic
4. Coordinate with training parameters

### Dependencies

**Core Dependencies:**
- ROS 2 Jazzy
- Gazebo Harmonic
- Python packages: numpy, torch, colcon-common-extensions
- ROS packages: ros_gz_bridge, ros_gz_sim

**Build System:**
- colcon for workspace building
- setuptools for Python package management
- ament_python for ROS 2 integration

### Configuration Management

The system uses a hierarchical configuration approach:
1. **Launch arguments** in training_launch.py (highest priority)
2. **Environment variables** in env_defaults.sh and config files
3. **Node parameters** with default values (lowest priority)

All bash scripts support optional configuration file arguments for easy experimentation.

### Troubleshooting

**Common Issues:**
- Gazebo startup delays: Use initial_reset_delay_sec parameter
- Collision detection problems: Check collision_distance and capture_radius
- Training instability: Adjust learning rates, exploration noise, or buffer capacity
- Memory issues: Reduce buffer_capacity or batch_size for large-scale training

**Debug Commands:**
```bash
# Check ROS 2 topics
ros2 topic list
ros2 topic echo /rl/episode_metrics

# Monitor node status
ros2 node list
ros2 param list /node_name

# Check Gazebo models
gz model --list
```