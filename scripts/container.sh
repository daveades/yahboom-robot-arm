#!/usr/bin/env bash
# Docker container for the DOFBOT stack.
#
# Usage:
#   scripts/container.sh            # start it if not running
#   scripts/container.sh status
#   scripts/container.sh stop

set -u
source "$(dirname -- "${BASH_SOURCE[0]:-$0}")/host_lib.sh"

cmd=${1:-start}

case "$cmd" in
    start)
        say "Docker container '$CONTAINER'"
        if container_running; then
            ok "already running"
            exit 0
        fi
        if ! docker start "$CONTAINER" >/dev/null 2>&1; then
            fail "could not start container — is Docker Desktop running?"
            exit 1
        fi
        ok "started"
        ;;
    stop)
        say "Stopping container '$CONTAINER'"
        docker stop "$CONTAINER" >/dev/null 2>&1 && ok "stopped" || warn "was not running"
        ;;
    status)
        if container_running; then ok "container: running"; else fail "container: NOT running"; exit 1; fi
        ;;
    *)
        echo "Usage: $0 [start|stop|status]"; exit 1 ;;
esac
