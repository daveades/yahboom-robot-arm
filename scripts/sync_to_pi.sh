#!/usr/bin/env bash
set -euo pipefail

PI_HOST=${PI_HOST:-ubuntu@192.168.82.50}
PI_KEY=${PI_KEY:-$HOME/.ssh/raspi}
PI_BASE=${PI_BASE:-/home/ubuntu/yahboom-robot-arm}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_SRC="$REPO_ROOT/dofbot_ros2_ws/src"

if [ "$#" -eq 0 ]; then
  PKGS=(dofbot_description dofbot_driver dofbot_moveit_config dofbot_ros2_control dofbot_vision dofbot_bringup)
else
  PKGS=("$@")
fi

# Validate packages
for pkg in "${PKGS[@]}"; do
  src="$WS_SRC/$pkg"
  if [ ! -d "$src" ]; then
    echo "Missing package: $src" >&2
    exit 1
  fi
done

echo "Syncing: ${PKGS[*]} -> $PI_HOST"

# Use a single SSH session to avoid repeated passphrase prompts.
tar -C "$WS_SRC" -cz "${PKGS[@]}" | ssh -i "$PI_KEY" "$PI_HOST" \
  "tar -C '$PI_BASE/dofbot_ros2_ws/src' -xz"

echo "Done."
