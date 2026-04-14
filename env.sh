#!/usr/bin/env bash

# Use project-local Python environment.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/venv/bin/activate"

# Load ROS 2 Jazzy environment.
# Aready sourced by ~/.bashrc, but we need to source it here for the case when this script is run in a non-interactive shell.
source /opt/ros/jazzy/setup.bash

# Load this workspace install environment if it exists.
if [ -f "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash" ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install/setup.bash"
fi

# Set ROS_DOMAIN_ID to 0 for all nodes in this workspace.
export ROS_DOMAIN_ID=0

# For the case when run in WSL,the fastdds would be not available.Set RMW_IMPLEMENTATION to cyclonedds for all nodes in this workspace.
# If you have fastdds installed and want to use it, please set RMW_IMPLEMENTATION to rmw_fastrtps_cpp.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# You should run apt install ros-jazzy-rmw-cyclonedds-cpp first to install cyclonedds for ROS 2 Jazzy. If you have already installed it, please ignore this message.
