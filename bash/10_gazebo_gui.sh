#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${1:-}"
source "${SCRIPT_DIR}/_load_config.sh" "${CONFIG_FILE}"

GZ_ARGS=(
  -r                      # 可选: 启动后立即运行，不暂停
  -v 3                    # 可选: 日志级别，常用 2/3/4
)

if [[ "${GAZEBO_GUI}" != "true" ]]; then
  GZ_ARGS+=( -s )
fi

GZ_ARGS+=( "${WORLD_SDF}" )

echo "[10_gazebo_gui] WORLD_NAME=${WORLD_NAME}, WORLD_SDF=${WORLD_SDF}, GAZEBO_GUI=${GAZEBO_GUI}"
exec gz sim "${GZ_ARGS[@]}"
