#!/usr/bin/env bash
# Camera bridge, FOREGROUND: reads the ffmpeg network stream from Windows
# and publishes /image_raw. All output streams to this terminal.
#
# Prerequisite — on WINDOWS (PowerShell), start the streamer first:
#   scripts\windows\stream_camera.ps1
# (use -ListDevices to find the camera name if "USB Camera" is wrong)
#
# Usage (inside the container):
#   scripts/camera.sh
#   scripts/camera.sh --url http://<windows-ip>:8090/cam.mjpg
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/container_lib.sh"

URL="http://host.docker.internal:8090/cam.mjpg"
prev=""
for arg in "$@"; do
    [ "$prev" = "--url" ] && URL=$arg
    prev=$arg
done

echo "Reading stream: $URL"
echo "(if it can't connect: is the ffmpeg streamer running on Windows,"
echo " and is port 8090 allowed through the Windows firewall?)"
echo

exec ros2 run dofbot_vision stream_camera --ros-args -p url:="$URL"
