#!/usr/bin/env python3
"""Probe the DOFBOT SE expansion board for undocumented reply opcodes.

Goal: find a servo position READ command. The write protocol is
    FF FC [len] [cmd] [payload...] [csum]
where len counts every byte from the len byte through the checksum, and
csum = sum(len..payload) & 0xFF. Known write commands: 0x10+id (single
servo), 0x1D (all six). This script sweeps candidate command bytes with
small payload variants and reports every command that makes the board
send ANYTHING back.

SAFETY / expectations:
  * Write commands (0x10..0x16, 0x1D) are skipped, so the pose should
    not change - but unknown opcodes may beep the buzzer, toggle LEDs,
    or in the worst case disable servo torque (arm slumps). Keep the
    arm in a safe pose (K1 center) and be ready to support it.
  * Stop the driver first (Ctrl-C scripts/driver.sh) so the port is
    quiet during the probe.

Usage (inside the container, arm attached):
    python3 tools/probe_protocol.py --yes                 # full sweep
    python3 tools/probe_protocol.py --yes --start 0x30 --end 0x40
    python3 tools/probe_protocol.py --yes --cmd 0x31      # single opcode
Every packet sent and every reply is appended to the log file.
"""
import argparse
import glob
import sys
import time

import serial

HEADER, DEVICE_ID = 0xFF, 0xFC
WRITE_CMDS = set(range(0x10, 0x17)) | {0x1D}  # never send: they move servos


def build_packet(cmd: int, payload: list) -> bytes:
    length = 1 + 1 + len(payload) + 1  # len + cmd + payload + csum
    body = [length, cmd] + payload
    csum = sum(body) & 0xFF
    return bytes([HEADER, DEVICE_ID] + body + [csum])


def auto_int(v: str) -> int:
    return int(v, 0)  # accepts 0x31 or 49


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None, help="default: first /dev/ttyUSB*")
    ap.add_argument("--start", type=auto_int, default=0x00)
    ap.add_argument("--end", type=auto_int, default=0xFF)
    ap.add_argument("--cmd", type=auto_int, default=None,
                    help="probe a single command byte instead of a range")
    ap.add_argument("--delay", type=float, default=0.05,
                    help="seconds between packets (default 0.05)")
    ap.add_argument("--reply-wait", type=float, default=0.08,
                    help="seconds to wait for a reply (default 0.08)")
    ap.add_argument("--log", default="probe_log.txt")
    ap.add_argument("--yes", action="store_true",
                    help="I read the safety note above")
    args = ap.parse_args()

    if not args.yes:
        print(__doc__)
        print("Re-run with --yes after reading the safety notes.", file=sys.stderr)
        return 1

    port = args.port or next(iter(sorted(glob.glob("/dev/ttyUSB*"))), None)
    if not port:
        print("No /dev/ttyUSB* device.", file=sys.stderr)
        return 1
    ser = serial.Serial(port, 115200, timeout=args.reply_wait)
    log = open(args.log, "a")
    log.write(f"\n=== probe run {time.strftime('%F %T')} on {port} ===\n")

    cmds = [args.cmd] if args.cmd is not None else [
        c for c in range(args.start, args.end + 1) if c not in WRITE_CMDS
    ]
    # payload variants: none, servo id 1, servo id + zero byte
    variants = [[], [0x01], [0x01, 0x00]]

    hits = []
    print(f"Probing {len(cmds)} commands x {len(variants)} payload variants "
          f"on {port} (skipping write opcodes)...")
    for cmd in cmds:
        for payload in variants:
            pkt = build_packet(cmd, payload)
            ser.reset_input_buffer()
            ser.write(pkt)
            reply = ser.read(64)
            log.write(f"cmd=0x{cmd:02X} payload={bytes(payload).hex() or '-'} "
                      f"sent={pkt.hex()} reply={reply.hex() or '-'}\n")
            if reply:
                hit = (f"cmd=0x{cmd:02X} payload=[{bytes(payload).hex() or '-'}] "
                       f"-> reply {len(reply)}B: {reply.hex(' ')}")
                hits.append(hit)
                print("  HIT:", hit)
            time.sleep(args.delay)
    log.close()

    print(f"\nDone. {len(hits)} replying command(s). Full log: {args.log}")
    if hits:
        print("Next: re-send each hit with different servo ids in the payload "
              "and check whether the reply bytes track the arm's real pose "
              "(move a joint by hand with torque off / power cycled).")
    else:
        print("No replies. The read command may need a different framing, "
              "or this firmware simply never transmits.")
    return 0


if __name__ == "__main__":
    main()
