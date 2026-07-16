#!/usr/bin/env bash
# One-command bringup for the DOFBOT stack (WSL2 + Docker, no Pi).
#
# Brings up, in order, skipping anything already running:
#   1. Docker container `dofbot`
#   2. USB serial passthrough (usbipd attach -> /dev/ttyUSB0 in container)
#   3. pi_control.launch.py  (driver + ros2_control + controllers)
#   4. moveit_pc.launch.py   (move_group + RViz)
#
# Usage:
#   scripts/start_arm.sh                 # start everything that isn't up
#   scripts/start_arm.sh -i              # interactive: decide per step
#   scripts/start_arm.sh --no-rviz       # MoveIt without RViz
#   scripts/start_arm.sh --startup-time 6000   # slower driver startup sweep
#   scripts/start_arm.sh status          # show what's running
#   scripts/start_arm.sh stop            # stop pi_control + MoveIt
#   scripts/start_arm.sh restart         # stop, then start
#   scripts/start_arm.sh logs            # tail pi_control + MoveIt logs

set -u

CONTAINER=dofbot
WS=/root/yahboom-robot-arm
SRC_ENV="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash"
LOG_DIR_C=$WS/runs/logs
REPO_DIR="$(cd "$(dirname -- "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
LOG_DIR_H="$REPO_DIR/runs/logs"

INTERACTIVE=0
USE_RVIZ=true
STARTUP_TIME=4000
CMD=start

for arg in "$@"; do
    case "$arg" in
        start|stop|status|logs|restart) CMD=$arg ;;
        -i|--interactive) INTERACTIVE=1 ;;
        --no-rviz) USE_RVIZ=false ;;
        --startup-time) : ;;  # value handled below
        *)
            if [[ "$arg" =~ ^[0-9]+$ ]]; then STARTUP_TIME=$arg
            else echo "Unknown option: $arg"; exit 1; fi ;;
    esac
done

# ---------- helpers ----------

say()  { echo -e "\033[1;36m==>\033[0m $*"; }
ok()   { echo -e "    \033[1;32m[ok]\033[0m $*"; }
warn() { echo -e "    \033[1;33m[!!]\033[0m $*"; }
fail() { echo -e "    \033[1;31m[XX]\033[0m $*"; }

# ask "question" -> 0 if yes. Non-interactive mode always says yes.
ask() {
    [ "$INTERACTIVE" -eq 0 ] && return 0
    local ans
    read -r -p "    Run this? [Y/n] " ans
    case "$ans" in n|N|no|NO) return 1 ;; *) return 0 ;; esac
}

in_container() { docker exec "$CONTAINER" bash -c "$1" 2>/dev/null; }

container_running() {
    docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"
}

have_tty() { in_container "test -e /dev/ttyUSB0"; }

# Note: [b]racketed patterns stop pgrep/pkill -f from matching the
# 'bash -c' wrapper that carries the pattern in its own command line.
pi_control_running() { in_container "pgrep -f '[d]ofbot_driver' >/dev/null || pgrep -x ros2_control_node >/dev/null"; }

moveit_running() { in_container "pgrep -x move_group >/dev/null"; }

controllers_active() {
    # daemon-free: the arm_controller topics only exist once the
    # controller is spawned ('ros2 control' CLI needs the ros2 daemon,
    # which is often wedged in this container)
    in_container "$SRC_ENV; timeout 8 ros2 topic list --no-daemon 2>/dev/null" \
        | grep -q "arm_controller"
}

# ---------- steps ----------

step_container() {
    say "Step 1/4: Docker container '$CONTAINER'"
    if container_running; then
        ok "already running"
        return 0
    fi
    warn "not running"
    ask "start container" || return 1
    if ! docker start "$CONTAINER" >/dev/null 2>&1; then
        fail "could not start container — is Docker Desktop running?"
        return 1
    fi
    ok "started"
}

step_usb() {
    say "Step 2/4: USB serial passthrough"
    if have_tty; then
        ok "/dev/ttyUSB0 present in container"
        return 0
    fi
    warn "/dev/ttyUSB0 missing — need usbipd attach"
    ask "attach the arm's USB serial to WSL" || return 1

    local connected busid state out
    # only the "Connected:" section has busids; "Persisted:" has GUIDs
    connected=$(usbipd.exe list 2>/dev/null | tr -d '\r' \
        | awk '/^Connected:/{f=1;next} /^Persisted:/{f=0} f')
    busid=$(echo "$connected" | grep -iE 'USB[ -]?Serial|CH340|CP210' \
        | awk '$1 ~ /^[0-9]+-[0-9]+$/ {print $1; exit}')
    if [ -z "$busid" ]; then
        fail "no USB-serial device found in 'usbipd list' — is the arm plugged in and powered?"
        return 1
    fi
    state=$(echo "$connected" | grep "^$busid " | grep -oiE 'Not shared|Shared|Attached' | head -1)
    say "found arm serial at busid $busid (state: $state)"
    if [ "$state" = "Attached" ]; then
        warn "already attached but no /dev/ttyUSB0 — detaching and re-attaching"
        usbipd.exe detach --busid "$busid" 2>/dev/null
        sleep 1
    fi
    if ! out=$(usbipd.exe attach --wsl --busid "$busid" 2>&1); then
        echo "$out" | tr -d '\r' | sed 's/^/    /'
        fail "attach failed. If the device is 'Not shared', run once in an ADMIN PowerShell:"
        fail "    usbipd bind --busid $busid"
        return 1
    fi
    # wait for the device node to appear in the container
    local i
    for i in $(seq 1 15); do
        have_tty && { ok "/dev/ttyUSB0 is up"; return 0; }
        sleep 1
    done
    fail "attach succeeded but /dev/ttyUSB0 never appeared in the container"
    return 1
}

step_pi_control() {
    say "Step 3/4: pi_control (driver + controllers)"
    if pi_control_running; then
        ok "already running"
        if [ "$INTERACTIVE" -eq 1 ]; then
            local ans
            read -r -p "    Restart it (needed after a driver rebuild)? [y/N] " ans
            case "$ans" in y|Y|yes|YES) stop_pi_control ;; *) return 0 ;; esac
        else
            return 0
        fi
    fi
    ask "launch pi_control" || return 1

    echo
    echo "    >>> Press K1 on the arm now so the servos are at their centered"
    echo "    >>> reference pose, then continue."
    if [ "$INTERACTIVE" -eq 1 ]; then
        read -r -p "    Press Enter when done... " _
    else
        sleep 3
    fi

    in_container "mkdir -p $LOG_DIR_C"
    docker exec -d "$CONTAINER" bash -c \
        "$SRC_ENV; exec ros2 launch dofbot_bringup pi_control.launch.py startup_time_ms:=$STARTUP_TIME >> $LOG_DIR_C/pi_control.log 2>&1"
    say "waiting for controllers to come up..."
    local i
    for i in $(seq 1 30); do
        controllers_active && { ok "arm_controller active"; return 0; }
        sleep 1
    done
    fail "controllers did not become active within 30s — check: $0 logs"
    return 1
}

step_moveit() {
    say "Step 4/4: MoveIt (move_group + RViz)"
    if moveit_running; then
        ok "already running"
        return 0
    fi
    if [ "$INTERACTIVE" -eq 1 ]; then
        local ans
        read -r -p "    Start RViz with MoveIt? [Y/n] " ans
        case "$ans" in n|N|no|NO) USE_RVIZ=false ;; esac
    fi
    ask "launch MoveIt (use_rviz:=$USE_RVIZ)" || return 1
    docker exec -d "$CONTAINER" bash -c \
        "$SRC_ENV; exec ros2 launch dofbot_bringup moveit_pc.launch.py use_rviz:=$USE_RVIZ >> $LOG_DIR_C/moveit.log 2>&1"
    say "waiting for move_group..."
    local i
    for i in $(seq 1 30); do
        moveit_running && { ok "move_group is up"; return 0; }
        sleep 1
    done
    fail "move_group did not start within 30s — check: $0 logs"
    return 1
}

# ---------- commands ----------

stop_pi_control() {
    in_container 'pkill -INT -f "[r]os2 launch dofbot_bringup pi_control" ; pkill -INT -f "[d]ofbot_driver" ; pkill -INT -x ros2_control_node ; pkill -INT -x robot_state_publisher' || true
    sleep 2
}

do_stop() {
    say "Stopping MoveIt and pi_control"
    if ! container_running; then
        warn "container not running — nothing to stop"
        return 0
    fi
    in_container 'pkill -INT -f "[r]os2 launch dofbot_bringup moveit_pc" ; pkill -INT -x move_group ; pkill -INT -x rviz2' || true
    stop_pi_control
    sleep 1
    # force-kill stragglers
    in_container 'pkill -9 -f "[r]os2 launch dofbot_bringup" ; pkill -9 -x move_group ; pkill -9 -x rviz2 ; pkill -9 -f "[d]ofbot_driver" ; pkill -9 -x ros2_control_node ; pkill -9 -x robot_state_publisher' || true
    ok "stopped"
}

do_status() {
    say "Status"
    if container_running; then ok "container: running"; else fail "container: NOT running"; return; fi
    if have_tty; then ok "serial: /dev/ttyUSB0 present"; else fail "serial: /dev/ttyUSB0 MISSING (driver would be deaf)"; fi
    if pi_control_running; then
        if controllers_active; then ok "pi_control: running, arm_controller active"
        else warn "pi_control: processes up but arm_controller not active"; fi
    else
        fail "pi_control: not running"
    fi
    if moveit_running; then ok "moveit: move_group running"; else fail "moveit: not running"; fi
}

do_logs() {
    say "Tailing logs (Ctrl-C to quit)"
    in_container "mkdir -p $LOG_DIR_C; touch $LOG_DIR_C/pi_control.log $LOG_DIR_C/moveit.log"
    tail -n 30 -f "$LOG_DIR_H/pi_control.log" "$LOG_DIR_H/moveit.log"
}

do_start() {
    step_container || exit 1
    step_usb       || exit 1
    step_pi_control || exit 1
    step_moveit    || exit 1
    echo
    say "All up. Next steps:"
    echo "    hover test:  docker exec -it $CONTAINER bash -c '$SRC_ENV; python3 $WS/tools/hover_test.py --a1 0.085 0.155 --square 0.04375 --yaw -90'"
    echo "    status:      $0 status"
    echo "    logs:        $0 logs"
    echo "    stop:        $0 stop"
}

# --startup-time takes a value: re-scan argv pairwise
prev=""
for arg in "$@"; do
    if [ "$prev" = "--startup-time" ]; then STARTUP_TIME=$arg; fi
    prev=$arg
done

case "$CMD" in
    start)   do_start ;;
    stop)    do_stop ;;
    status)  do_status ;;
    logs)    do_logs ;;
    restart) do_stop; echo; do_start ;;
esac
