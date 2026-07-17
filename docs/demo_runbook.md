# Chess Demo Runbook

From cold machine to playing the robot. Terminals are numbered — the
driver and MoveIt each keep one so their logs stay visible.

The robot plays **White** on the printed 26 mm board and can physically
reach roughly **ranks 1–4**; the game restricts its move choice to what
it can execute and asks for a hand otherwise. You move your own pieces
and type your moves.

## 0. One-time prep

1. Generate and print the board (inside the container):
   ```bash
   python3 tools/gen_board.py --paper a3 --marker-mm 20 --border-mm 10 --out /root/yahboom-robot-arm/board_a3
   ```
   Print `board_a3.pdf` at **100% / Actual size**, ruler-check a square
   measures 26 mm.
2. Trim the blank paper outside the gray border line (untrimmed, the
   near margin collides with the robot's base).
3. Glue/tape to cardboard so it lies flat.

## A. Power and host (Windows + WSL)

1. Power the arm from its **DC supply** (not USB power) and switch it
   on. Stand the arm roughly upright.
2. Start **Docker Desktop**, wait until it reports running.
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
Confirms `/dev/ttyUSB*` exists, prompts for the K1/center pose (press
K1 on the expansion board, then confirm). Watch for the
"Connected to /dev/ttyUSB0" line — no line, no arm.

Terminal 2 — MoveIt:
```bash
docker exec -it dofbot bash
scripts/moveit.sh          # RViz opens; --no-rviz to skip it
```

Health check from any terminal: `scripts/status.sh`.

**First-motion smoke test:** in RViz plan a *small* arm move at low
velocity scaling (0.1–0.2) and Execute. Smooth motion and no USB drop
in Terminal 1's log → proceed.

## C. Board placement and calibration (first time per board)

1. Place the board **ARM SIDE toward the robot**, centered on the arm's
   forward (K1) direction, with the **first grid line ≈ 12 cm** from the
   base's rotation axis (that puts a1's center at 13 cm; measure from
   the rotation axis, not the housing edge; ±1 cm is fine):

   ```
        a8 ................ h8      far side (yours)
        a1 ................ h1      rank 1 — ARM SIDE
                 ~12 cm
              [robot base]          centerline hits the d/e boundary
   ```
2. Set `config/board.yaml` to the nominal print numbers:
   `a1: [0.130, 0.091]`, `square: 0.026`, `yaw_deg: -90`,
   `mirror: false`.
3. Terminal 3 — check where the arm thinks the squares are:
   ```bash
   cd /root/yahboom-robot-arm
   python3 tools/hover_test.py --gripper -1.0 a1 h1 d4 e3
   ```
   Off-center → shift `a1` in board.yaml by the offset and re-run.
   Files/ranks swapped or mirrored → adjust `yaw_deg` / `mirror`.
4. Find the real heights (sim values are wrist-frame approximations):
   step `hover_test.py --z` downward over an empty square until the
   fingertips are at piece-gripping height → **grasp_z**. Travel height
   **hover_z** must let a carried piece clear standing pieces.
5. Gripper: put a piece on d3, descend over it, try `--gripper -0.6`,
   `-0.8`, ... until it holds firm without straining → **grip-closed**.
6. Confirm the playable zone with the calibrated numbers:
   ```bash
   python3 tools/reach_check.py --hover-z <H> --grasp-z <G>
   ```
   Expect ranks 1–4-ish fully `#`.

## D. Play

Set up all 32 pieces (robot = White on the near ranks), then:

```bash
python3 tools/chess_game.py --hover-z <H> --grasp-z <G> --grip-closed <C> --skill 3
```

- Startup maps reachable squares once (cached in
  `runs/reach_cache.json`; auto-invalidates when geometry/heights
  change) and prints `Arm can play on N/64 squares`.
- Type your moves as SAN (`e5`, `Nf6`) or UCI (`e7e5`); `quit` resigns.
- `OUT OF REACH` → make the robot's announced move for it, press Enter.
- `--skill 0..20` sets strength, `--move-time 3` slows the arm for
  showmanship, `--fen` resumes a position.

## Sim rehearsal (no hardware)

```bash
# Terminal 1
docker exec -it dofbot bash -c 'scripts/sim.sh'
# Terminal 2
docker exec -it dofbot bash
cd /root/yahboom-robot-arm
python3 tools/chess_game.py --self-play --hover-z 0.06 --grasp-z 0.05
```
Never run sim.sh while driver.sh/moveit.sh are up.

## Troubleshooting

- **ROS happy, arm deaf** — Terminal 1: serial errors? `ls /dev/ttyUSB*`
  empty? Re-run `scripts/usb.sh` on the host.
- **USB drops mid-game** — the watcher re-attaches and the driver
  reconnects automatically, but drops mean power trouble: check the DC
  supply and cable.
- **A square consistently off** — the board moved; re-seat it on its
  traced outline or redo step C-3.
- **Controllers/actions missing** — `scripts/status.sh`, then restart
  driver.sh (hardware) or sim.sh (sim); never both at once.
