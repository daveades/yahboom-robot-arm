#!/usr/bin/env bash
# Camera->base homography calibration from the chessboard, FOREGROUND.
# Needs the camera bridge running (scripts/camera.sh) OR an --image file.
# Board must be EMPTY and fully visible.
#
# The board model comes from config/board.yaml (single source of truth);
# any flag you pass overrides it.
#
# Usage (inside the container):
#   scripts/homography.sh                          # grab from /image_raw
#   scripts/homography.sh --rotate 90              # fix orientation
#   scripts/homography.sh --image /path/frame.png  # from a file
#   scripts/homography.sh --a1 0.09 0.16           # override board model
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/container_lib.sh"

exec python3 "$WS/tools/calibrate_camera.py" \
    --annotate "$WS/runs/calib_check.png" \
    "$@"
