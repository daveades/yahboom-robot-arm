#!/usr/bin/env bash
# USB serial passthrough: usbipd attach (Windows -> WSL -> container).
#
# Usage:
#   scripts/usb.sh            # attach if /dev/ttyUSB0 is missing
#   scripts/usb.sh status

set -u
source "$(dirname -- "${BASH_SOURCE[0]:-$0}")/host_lib.sh"

cmd=${1:-attach}

usb_status() {
    if have_tty; then
        ok "serial: $(in_container "ls /dev/ttyUSB*" | tr '\n' ' ')present in container"
    else
        fail "serial: no /dev/ttyUSB* in container"
        return 1
    fi
}

usb_attach() {
    say "USB serial passthrough"
    if have_tty; then
        ok "/dev/ttyUSB0 present in container"
        return 0
    fi
    warn "/dev/ttyUSB0 missing — need usbipd attach"

    local connected busid state
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
    # --auto-attach keeps a watcher running that re-attaches the device
    # whenever it re-enumerates (the CH340 drops out on cable glitches /
    # board brownouts). It blocks, so run it in the background.
    ensure_logdir
    nohup usbipd.exe attach --wsl --busid "$busid" --auto-attach \
        >> "$LOG_DIR_H/usbipd.log" 2>&1 &
    disown
    # wait for the device node to appear in the container
    local i
    for i in $(seq 1 15); do
        have_tty && { ok "/dev/ttyUSB0 is up (auto-attach watcher running)"; return 0; }
        sleep 1
    done
    fail "attach did not produce /dev/ttyUSB0 in the container. Check $LOG_DIR_H/usbipd.log"
    fail "If the device is 'Not shared', run once in an ADMIN PowerShell:  usbipd bind --busid $busid"
    return 1
}

case "$cmd" in
    attach) usb_attach ;;
    status) usb_status ;;
    *) echo "Usage: $0 [attach|status]"; exit 1 ;;
esac
