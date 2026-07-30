# Chess Demo Runbook

Cold machine to playing the robot, in order. Background and calibration
theory are in [user_manual.md §12](user_manual.md); this is the checklist.

The robot plays **White** on the printed 26 mm board and can physically
reach roughly **ranks 1–4**. It restricts its own move choice to squares it
can execute and asks you to play the rest by hand. You move your own pieces
and type your moves.

## A. Power and host (Windows + WSL)

1. Power the arm from its **DC supply** and switch it on. Stand it roughly
   upright.
2. Start **Docker Desktop**; wait until it reports running.
3. In a WSL terminal:
   ```bash
   cd ~/ros2_ws/yahboom-robot-arm
   scripts/container.sh          # start the dofbot container
   scripts/usb.sh                # attach the arm's USB + auto-reattach watcher
   ```
   `usb.sh` should report the CH340 attached. If binding fails, run the
   bind once from an **admin** PowerShell as the script instructs.

## B. Robot stack (two container terminals)

Terminal 1 — driver:
```bash
docker exec -it dofbot bash
scripts/driver.sh
```
It confirms `/dev/ttyUSB*` exists and prompts for the K1/center pose.
Watch for `Connected to /dev/ttyUSB0` — no line, no arm.

Terminal 2 — MoveIt:
```bash
docker exec -it dofbot bash
scripts/moveit.sh          # RViz opens; --no-rviz to skip it
```

Health check from any terminal: `scripts/status.sh`.

**First-motion smoke test:** in RViz, plan a *small* arm move at velocity
scaling 0.1–0.2 and Execute. Smooth motion and no USB drop in terminal 1
→ proceed.

## C. Board (first time per board only)

Print and place the board per §12.2–§12.3, then run the calibration loop in
§12.4 until `hover_test.py` lands centred on the squares you check. Verify
the playable zone with the numbers you ended up with:

```bash
python3 tools/reach_check.py --hover-z <H> --grasp-z <G>
```

Expect ranks 1–4 mostly `#`. Once `config/board.yaml` is calibrated, skip
straight from B to D on later sessions.

## D. Play

Set up all 32 pieces (robot = White on the near ranks), then in terminal 3:

```bash
cd /root/yahboom-robot-arm
python3 tools/chess_game.py --skill 3
```

The calibrated numbers are already the defaults — hover 0.10, grasp 0.053,
grip-closed −1.42, both heights measured at the fingertips. Override with
`--hover-z` / `--grasp-z` / `--grip-closed` only if the board or the pieces
change.

- Startup prints `Arm can play on N/64 squares`.
- Type moves as SAN (`e5`, `Nf6`) or UCI (`e7e5`); `quit` resigns.
- `OUT OF REACH` → make the robot's announced move for it, press Enter.
- `--skill 0..20` sets strength, `--move-time 3` slows the arm for
  showmanship, `--fen` resumes a position.

## Troubleshooting

Full tables in §14. The three that bite during a demo:

- **ROS happy, arm deaf** — terminal 1: serial errors? `ls /dev/ttyUSB*`
  empty? Re-run `scripts/usb.sh` on the host.
- **USB drops mid-game** — the watcher re-attaches and the driver
  reconnects, but repeated drops mean power trouble: check the DC supply
  and the cable.
- **One square consistently off** — the board moved. Re-seat it on its
  traced outline, or redo the `hover_test.py` step of §12.4.
