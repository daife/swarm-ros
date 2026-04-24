#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../env_defaults.sh"

PURSUER_ALGO="flocking"
PURSUER_TRAIN_MODE="eval_only"
PURSUER_WEIGHTS=""
EVADER_ALGO="ddpg"
EVADER_TRAIN_MODE="train_scratch"
EVADER_WEIGHTS=""
