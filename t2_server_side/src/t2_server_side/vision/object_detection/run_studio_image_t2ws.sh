#!/usr/bin/env bash
source /opt/ros/kilted/setup.bash || exit 1
set -euo pipefail
cd /ws
export PYTHONPATH="/ws/t2_server_side/src${PYTHONPATH:+:$PYTHONPATH}"

exec python -m t2_server_side.vision.object_detection.viewer \
  --studio-only --device "${T2_YOLO_DEVICE:-cpu}" "$@"
