#!/usr/bin/env bash
# Status of the whole DOFBOT stack, inside the container.
#
# Usage:
#   scripts/status.sh                # summary: serial, processes, key topics
#   scripts/status.sh nodes          # all nodes
#   scripts/status.sh topics         # all topics (with types)
#   scripts/status.sh services       # all services
#   scripts/status.sh actions        # all actions
#   scripts/status.sh controllers    # trajectory action servers (driver)
#   scripts/status.sh hz <topic>     # measure a topic's publish rate
#   scripts/status.sh echo <topic>   # print one message from a topic
#   scripts/status.sh all            # summary + nodes + topics + services
#
# Everything uses --no-daemon where possible (the ros2 CLI daemon wedges
# in this container); discovery takes a couple of seconds per section.
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/container_lib.sh"

ok()   { echo -e "  \033[1;32m[ok]\033[0m $*"; }
bad()  { echo -e "  \033[1;31m[--]\033[0m $*"; }
head_() { echo -e "\033[1;36m== $* ==\033[0m"; }

proc() {  # proc <label> <pgrep-args...>
    local label=$1; shift
    if pgrep "$@" >/dev/null 2>&1; then
        ok "$label (pid $(pgrep "$@" | head -1))"
    else
        bad "$label not running"
    fi
}

summary() {
    head_ "Serial"
    local tty
    tty=$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')
    if [ -n "$tty" ]; then ok "$tty"; else bad "no /dev/ttyUSB* — run scripts/usb.sh on the host"; fi

    head_ "Processes"
    proc "dofbot_driver        " -f '[d]ofbot_driver'
    # -f: pgrep -x compares the kernel comm name, truncated to 15 chars,
    # so long binary names like robot_state_publisher never match it.
    proc "robot_state_publisher" -f '[r]obot_state_publisher'
    proc "move_group           " -x move_group
    proc "rviz2                " -x rviz2
    proc "camera bridge        " -f '[s]tream_camera'

    head_ "Key topics (discovering...)"
    local topics t
    topics=$(timeout 10 ros2 topic list --no-daemon --include-hidden-topics 2>/dev/null)
    local c
    for c in arm gripper; do
        if echo "$topics" | grep -q "/${c}_controller/follow_joint_trajectory"; then
            ok "${c}_controller action server"
        elif pgrep -f '[d]ofbot_driver' >/dev/null 2>&1; then
            # The CLI's graph discovery misses hidden action topics at
            # times; the driver creates both servers unconditionally.
            ok "${c}_controller action server (inferred from driver; discovery flaked)"
        else
            bad "${c}_controller action server missing (driver not up?)"
        fi
    done
    for t in /joint_states /image_raw; do
        if echo "$topics" | grep -qx "$t"; then ok "$t"; else bad "$t missing"; fi
    done

    echo
    echo "More: $0 {nodes|topics|services|actions|controllers|all}"
    echo "      $0 hz /joint_states     $0 echo /joint_states"
}

case "${1:-summary}" in
    summary)  summary ;;
    nodes)
        head_ "Nodes";    timeout 10 ros2 node list --no-daemon 2>/dev/null ;;
    topics)
        head_ "Topics";   timeout 10 ros2 topic list -t --no-daemon 2>/dev/null ;;
    services)
        head_ "Services"; timeout 10 ros2 service list --no-daemon 2>/dev/null ;;
    actions)
        head_ "Actions";  timeout 10 ros2 action list 2>/dev/null \
            || echo "(action list needs the ros2 daemon; if it hangs: ros2 daemon stop)" ;;
    controllers)
        head_ "Controllers (action servers served by the driver)"
        timeout 10 ros2 topic list --no-daemon --include-hidden-topics 2>/dev/null \
            | grep -E '_controller/' || echo "no controller action topics — driver not up?" ;;
    hz)
        [ $# -ge 2 ] || { echo "usage: $0 hz <topic>" >&2; exit 1; }
        exec ros2 topic hz "$2" ;;
    echo)
        [ $# -ge 2 ] || { echo "usage: $0 echo <topic>" >&2; exit 1; }
        exec ros2 topic echo --once "$2" ;;
    all)
        summary; echo
        "$0" nodes; echo
        "$0" topics; echo
        "$0" services ;;
    *)
        echo "unknown section: $1" >&2
        echo "usage: $0 {summary|nodes|topics|services|actions|controllers|hz <t>|echo <t>|all}" >&2
        exit 1 ;;
esac
