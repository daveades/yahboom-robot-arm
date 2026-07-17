#!/usr/bin/env bash
# Shared config + helpers for the bringup scripts. Source, don't run:
#   source "$(dirname "$0")/host_lib.sh"

CONTAINER=dofbot
WS=/root/yahboom-robot-arm
LOG_DIR_C=$WS/runs/logs
REPO_DIR="$(cd "$(dirname -- "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
LOG_DIR_H="$REPO_DIR/runs/logs"

say()  { echo -e "\033[1;36m==>\033[0m $*"; }
ok()   { echo -e "    \033[1;32m[ok]\033[0m $*"; }
warn() { echo -e "    \033[1;33m[!!]\033[0m $*"; }
fail() { echo -e "    \033[1;31m[XX]\033[0m $*"; }

in_container() { docker exec "$CONTAINER" bash -c "$1" 2>/dev/null; }

# container creates the dir as root; open it up so the host-side usbipd
# watcher can log there too
ensure_logdir() { in_container "mkdir -p $LOG_DIR_C && chmod a+rwX $LOG_DIR_C"; }

container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"
}

# device can re-enumerate as ttyUSB1 after a drop; any ttyUSB counts
have_tty() { in_container "compgen -G '/dev/ttyUSB*' >/dev/null"; }

