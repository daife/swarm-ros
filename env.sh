#!/usr/bin/env bash

# Detect script directory (works for both bash and zsh when sourced)
if [ -n "${BASH_SOURCE[0]}" ]; then
    _env_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elif [ -n "${(%):-%x}" ]; then
    _env_script_dir="$(cd "$(dirname "${(%):-%x}")" && pwd)"
else
    _env_script_dir="$(cd "$(dirname "$0")" && pwd)"
fi

# Use project-local Python environment.
source "${_env_script_dir}/venv/bin/activate"

# Load ROS 2 Jazzy environment.
if [ -n "${ZSH_VERSION}" ]; then
    source /opt/ros/jazzy/setup.zsh
else
    source /opt/ros/jazzy/setup.bash
fi

# Load this workspace install environment if it exists.
if [ -n "${ZSH_VERSION}" ] && [ -f "${_env_script_dir}/install/setup.zsh" ]; then
    source "${_env_script_dir}/install/setup.zsh"
elif [ -f "${_env_script_dir}/install/setup.bash" ]; then
    source "${_env_script_dir}/install/setup.bash"
fi

# Set ROS_DOMAIN_ID to 0 for all nodes in this workspace.
export ROS_DOMAIN_ID=0

# For WSL, fastdds may be unavailable. Use cyclonedds instead.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

unset _env_script_dir
