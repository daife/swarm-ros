#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../env_defaults.sh"

PURSUER_TRAIN_MODE="eval_only"
EVADER_TRAIN_MODE="eval_only"
PURSUER_WEIGHTS="artifacts/checkpoints/pursuer_policy_best.pth"
EVADER_WEIGHTS="artifacts/checkpoints/pursuer_policy_evader_best.pth"
