#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_CONFIG="${SCRIPT_DIR}/env_defaults.sh"

# 用法：source _load_config.sh [config_file]
CONFIG_FILE="${1:-${DEFAULT_CONFIG}}"

if [[ "${CONFIG_FILE}" != /* ]]; then
  CONFIG_FILE="${PWD}/${CONFIG_FILE}"
fi

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "[config] file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

source "${CONFIG_FILE}"
