#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

GZ_ARGS=(
  -r                      # 可选: 启动后立即运行，不暂停
  -v 3                    # 可选: 日志级别，常用 2/3/4
  "${WORLD_SDF}"         # 可选: 指定 world 文件路径
)

echo "[10_gazebo_gui] WORLD_NAME=${WORLD_NAME}, WORLD_SDF=${WORLD_SDF}"
exec gz sim "${GZ_ARGS[@]}"
