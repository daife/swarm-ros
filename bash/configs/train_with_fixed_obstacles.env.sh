#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../env_defaults.sh"

WORLD_MODE="fixed"
SEED_MODE="single"
OBSTACLE_DENSITY="0.9"
ALWAYS_RESET_WORLD="false"

PURSUER_TRAIN_MODE="train_scratch"
EVADER_TRAIN_MODE="train_scratch"
PURSUER_WEIGHTS=""
EVADER_WEIGHTS=""
