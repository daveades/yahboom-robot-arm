#!/usr/bin/env bash
# Arm driver (control.launch.py: serial driver + robot_state_publisher),
# FOREGROUND. The driver serves the arm/gripper trajectory actions itself.
# All node output streams to this terminal; Ctrl-C stops everything.
#
# Usage (inside the container):
#   scripts/driver.sh                      # default 4000 ms startup sweep
#   scripts/driver.sh --startup-time 6000  # slower startup glide
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/container_lib.sh"

STARTUP_TIME=4000
prev=""
for arg in "$@"; do
    [ "$prev" = "--startup-time" ] && STARTUP_TIME=$arg
    prev=$arg
done

# The driver is open-loop: without a serial device every command is
# silently dropped while ROS reports success. Refuse to start deaf.
tty_dev=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -z "$tty_dev" ]; then
    echo "ERROR: no /dev/ttyUSB* in the container." >&2
    echo "On the WSL host run:  scripts/usb.sh   then retry here." >&2
    exit 1
fi
echo "Serial device: $tty_dev"
echo
echo ">>> Press K1 on the arm (servos to centered reference pose),"
echo ">>> then press Enter to launch."
read -r _

exec ros2 launch dofbot_bringup control.launch.py \
    port:="$tty_dev" startup_time_ms:="$STARTUP_TIME"
