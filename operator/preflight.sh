#!/usr/bin/env bash
# Read-only readiness check: environment, robot telemetry, LiDAR rates, IP fragment health.
set -eo pipefail
source "$(dirname "$0")/env.sh"
python3 "$OPERATOR_DIR/lite3ops.py" preflight --json "$LOG_ROOT/preflight-$(date +%Y%m%d-%H%M%S).json" "$@"
