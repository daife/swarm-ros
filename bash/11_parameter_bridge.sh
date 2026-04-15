#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

BRIDGE_ARGS=(
  "/world/${WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity"            # 可选: 用于运行时创建模型
  "/world/${WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose"        # 可选: 用于重置/控制实体位姿
)

for ((i=0; i<NUM_PURSUERS; i++)); do
  name="${PURSUER_PREFIX}_${i}"
  BRIDGE_ARGS+=(
    "/model/${name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"         # 可选: 里程计桥接
    "/model/${name}/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts" # 可选: 默认碰撞桥接
    "/world/${WORLD_NAME}/model/${name}/link/base_link/sensor/contact_sensor/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts" # 可选: 传感器碰撞桥接
  )
done

for ((i=0; i<NUM_EVADERS; i++)); do
  name="${EVADER_PREFIX}_${i}"
  BRIDGE_ARGS+=(
    "/model/${name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"         # 可选: 里程计桥接
    "/model/${name}/contacts@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts" # 可选: 默认碰撞桥接
    "/world/${WORLD_NAME}/model/${name}/link/base_link/sensor/contact_sensor/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts" # 可选: 传感器碰撞桥接
  )
done

echo "[11_parameter_bridge] WORLD_NAME=${WORLD_NAME}, agents=$((NUM_PURSUERS + NUM_EVADERS))"
exec ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_ARGS[@]}"
