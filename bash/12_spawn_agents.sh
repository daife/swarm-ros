#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

SWARM_SHARE="$(ros2 pkg prefix swarm_bringup)/share/swarm_bringup"
PURSUER_SDF="${PURSUER_SDF:-${SWARM_SHARE}/models/pursuer/model.sdf}"  # 可选: 追捕者模型路径
EVADER_SDF="${EVADER_SDF:-${SWARM_SHARE}/models/evader/model.sdf}"      # 可选: 逃避者模型路径
SPAWN_TIMEOUT_MS="${SPAWN_TIMEOUT_MS:-3000}"                            # 可选: 单次创建超时(ms)

spawn_one() {
  local model_name="$1"
  local model_sdf="$2"
  local x="$3"
  local y="$4"

  local req
  req="sdf_filename: \"${model_sdf}\" name: \"${model_name}\" pose: {position: {x: ${x}, y: ${y}, z: 0.70}}"

  echo "[12_spawn_agents] spawn ${model_name} at (${x}, ${y})"
  gz service -s "/world/${WORLD_NAME}/create" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout "${SPAWN_TIMEOUT_MS}" \
    --req "${req}"
}

idx=0
for ((i=0; i<NUM_PURSUERS; i++)); do
  row=$((idx / SPAWN_COLUMNS))
  col=$((idx % SPAWN_COLUMNS))
  x=$(awk "BEGIN{printf \"%.2f\", ${SPAWN_X0}+${col}*${SPAWN_SPACING}}")
  y=$(awk "BEGIN{printf \"%.2f\", ${SPAWN_Y0}+${row}*${SPAWN_SPACING}}")
  spawn_one "${PURSUER_PREFIX}_${i}" "${PURSUER_SDF}" "${x}" "${y}"
  idx=$((idx + 1))
done

for ((i=0; i<NUM_EVADERS; i++)); do
  row=$((idx / SPAWN_COLUMNS))
  col=$((idx % SPAWN_COLUMNS))
  x=$(awk "BEGIN{printf \"%.2f\", ${SPAWN_X0}+${col}*${SPAWN_SPACING}}")
  y=$(awk "BEGIN{printf \"%.2f\", ${SPAWN_Y0}+${row}*${SPAWN_SPACING}}")
  spawn_one "${EVADER_PREFIX}_${i}" "${EVADER_SDF}" "${x}" "${y}"
  idx=$((idx + 1))
done

echo "[12_spawn_agents] done"
