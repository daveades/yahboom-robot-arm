# Setup Guide — Yahboom DOFBOT ROS 2 Workspace

Get this project running in **your** environment, from zero to a working
MoveIt simulation, then (optionally) real hardware and the vision/picking
pipeline. Pick the path matching your machine and follow it top to bottom.

**What you need**

- Required: a PC running Windows 11 (WSL2), native Ubuntu 22.04, or anything
  that runs Docker
- For hardware stages: a Yahboom DOFBOT arm (USB serial) with its power
  supply, and its USB camera
- Disk: ~15 GB (ROS image + PyTorch)

---

## 1. Choose your path

| Your machine | Path |
|---|---|
| Windows + WSL2 (Docker Desktop) | **A** — fully tested, this is what the project was built on |
| Native Ubuntu 22.04 | **B** — simplest; no Docker layers |
| Other Linux / macOS via Docker | **C** — simulation & development only (see limits) |

---

## Path A — Windows + WSL2 + Docker

### A1. Prerequisites

1. Install WSL2 with an Ubuntu distro, and Docker Desktop.
2. Docker Desktop → **Settings → Resources → WSL integration** → enable for
   your distro → Apply & Restart. Verify inside WSL: `docker version`.

### A2. Clone and start the container

```bash
cd ~ && mkdir -p ros2_ws && cd ros2_ws
git clone <this-repo-url> yahboom-robot-arm

docker pull osrf/ros:humble-desktop-full

docker run -it --net=host --name dofbot \
  -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
  --mount type=bind,source=/mnt/wslg,target=/mnt/wslg \
  --mount type=bind,source=$HOME/ros2_ws/yahboom-robot-arm,target=/root/yahboom-robot-arm \
  --mount type=bind,source=/dev,target=/dev \
  --device-cgroup-rule='c 188:* rmw' \
  --device-cgroup-rule='c 166:* rmw' \
  --device-cgroup-rule='c 81:* rmw' \
  osrf/ros:humble-desktop-full
```

What the unusual flags do: the X11/WSLg mounts + `-e` vars let RViz open a
window on your desktop; the `/dev` mount + cgroup rules let USB serial
devices work even when hot-plugged. If RViz later renders garbage, add
`-e LIBGL_ALWAYS_SOFTWARE=1`.

Day-to-day: `docker start dofbot` then `docker exec -it dofbot bash` for
each terminal you need.

### A3. Install dependencies (inside the container)

```bash
apt update && apt install -y \
  ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
  python3-serial ros-humble-v4l2-camera ros-humble-rqt-image-view v4l-utils

# Pin Python packages BEFORE anything pip-related — see warning below
printf 'numpy<2\nopencv-python<4.11\nsetuptools<60\n' > /root/pip-constraints.txt
echo 'export PIP_CONSTRAINT=/root/pip-constraints.txt' >> /root/.bashrc
export PIP_CONSTRAINT=/root/pip-constraints.txt

# YOLO (only needed for the vision stages; ~2 GB, CPU-only wheel keeps it lean)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install ultralytics "numpy==1.26.4" "opencv-python==4.10.0.84"

# Convenience: auto-source ROS in every shell
echo 'source /opt/ros/humble/setup.bash; [ -f ~/yahboom-robot-arm/dofbot_ros2_ws/install/setup.bash ] && source ~/yahboom-robot-arm/dofbot_ros2_ws/install/setup.bash' >> ~/.bashrc
```

> ⚠️ **Do not skip the constraints file.** ROS Humble's `cv_bridge` and
> `colcon` break with numpy ≥2, opencv-python ≥5, or setuptools ≥60. Pip will
> try to "upgrade" you into breakage as a side effect of unrelated installs;
> the constraints file makes that impossible. If you ever see
> `_ARRAY_API not found`, a `KeyError: 16`, or `canonicalize_version()`
> errors, a pin was bypassed — re-run the pip line above (and
> `pip3 uninstall -y setuptools` to fall back to the distro's 59.6.0).

Verify the Python stack:

```bash
python3 -c "import numpy, cv2, ultralytics; from cv_bridge import CvBridge; \
  print('numpy', numpy.__version__, '| opencv', cv2.__version__, '| OK')"
```

Then snapshot so you never repeat this: **from WSL (not the container)**:

```bash
docker commit dofbot dofbot:setup
```

### A4. What this setup changes on your Windows machine

For transparency — the complete Windows-side footprint is four items, all
reversible:

| Change | Purpose | Undo |
|---|---|---|
| Docker Desktop WSL-integration toggle | `docker` CLI inside WSL | flip it off |
| usbipd-win (app + service) and per-device `bind` records | USB serial into WSL (section 3) | `usbipd unbind --all`, uninstall app |
| ffmpeg (winget) | camera network bridge (section 4) | `winget uninstall ffmpeg` |
| A firewall allow-rule for ffmpeg | lets WSL reach the video stream on port 8090 | remove in Windows Security → Allow an app through firewall |

No drivers are permanently replaced: while a device is *attached* to WSL it
disappears from Windows and returns on detach/unplug.

**Per-session Windows steps** (don't survive reboot/replug):
re-run `usbipd attach --wsl --busid <ID>` for the arm (or use
`--auto-attach`), and restart the ffmpeg streamer if using the camera. If
the arm ever "stops responding", check the attachment first:
`ls /dev/ttyUSB*` in WSL.

### A5. Build → jump to [section 2](#2-build-and-first-run-all-paths)

---

## Path B — Native Ubuntu 22.04

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop ros-humble-moveit \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  python3-serial ros-humble-v4l2-camera ros-humble-rqt-image-view v4l-utils \
  python3-pip git

cd ~ && git clone <this-repo-url> yahboom-robot-arm

# YOLO deps in a venv that can still see ROS packages:
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages ~/venvs/ros2_yolo
source ~/venvs/ros2_yolo/bin/activate
pip install -U pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics "numpy<2" "opencv-python<5"

# Serial port access (one-time, then log out/in):
sudo usermod -aG dialout $USER
```

Native advantages: the arm is just `/dev/ttyUSB0` when plugged in, and the
camera works directly with `v4l2_camera` — skip all usbipd/ffmpeg-bridge
steps below. Continue to [section 2](#2-build-and-first-run-all-paths).

---

## Path C — Other platforms via Docker

Use the Path A container command minus the WSLg-specific mounts; provide GUI
access per your platform (Linux: mount `/tmp/.X11-unix` + `xhost +local:`;
macOS: XQuartz or run RViz-less).

**Hard limits:** Docker on macOS/Windows-without-WSL cannot pass USB devices
through at all — simulation and code development work fine, but real
hardware needs a Linux host (or the Raspberry Pi split described in
[getting_started.md](getting_started.md)).

---

## 2. Build and first run (all paths)

```bash
source /opt/ros/humble/setup.bash
cd <repo>/dofbot_ros2_ws        # ALWAYS build here — one workspace, one install tree
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Smoke test — simulation (no hardware needed)

```bash
ros2 launch dofbot_moveit_config demo.launch.py
```

RViz opens with the robot model. In the **MotionPlanning** panel → Planning
tab: set Planning Group to `arm`, pick Goal State `<random valid>` (or drag
the orange end-effector marker — position only; orientation is ignored on
this 5-DOF arm), click **Plan & Execute**. If the virtual arm moves, your
environment is correct. **This is the checkpoint — don't proceed to hardware
until it passes.**

---

## 3. Real hardware — the arm

Physical prep, every session: **pose the arm straight up** (all joints
centered) before starting anything — the driver assumes this starting pose —
and connect the arm's own power supply; USB alone can't drive the servos.

### WSL2 only: attach the serial adapter

In PowerShell (admin needed once, for `bind`):

```powershell
winget install usbipd            # one-time
usbipd list                      # find the arm: "USB Serial", VID:PID 1a86:7523
usbipd bind --busid <BUSID>      # one-time, admin
usbipd attach --wsl --busid <BUSID>   # every replug/reboot (--auto-attach to persist)
```

Verify in the container/WSL: `ls /dev/ttyUSB*` → `/dev/ttyUSB0`.

### Start the stack (two terminals)

```bash
# Terminal 1 — driver + ros2_control + controllers:
ros2 launch dofbot_bringup control.launch.py     # port defaults to /dev/ttyUSB0

# Terminal 2 — verify, then MoveIt + RViz:
ros2 control list_controllers      # all three must say "active"
ros2 launch dofbot_bringup moveit.launch.py
```

Plan & Execute in RViz now moves the real arm. **Set Velocity Scaling to
0.1–0.2 for your first motions**, keep the area clear, and remember there is
no obstacle model — nothing stops a plan from passing through your desk.

---

## 4. Vision

### Camera input — depends on your path

**Native Linux:** plug in the camera and run:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p image_size:=[640,480]
```

**WSL2: the camera cannot be attached via usbipd** (USB webcam streaming —
isochronous transfers — doesn't survive USB/IP; it will enumerate but never
deliver frames). The camera must stay **owned by Windows** — if you attached
it while experimenting, `usbipd detach --busid <ID>` first. Then use the
network bridge:

1. Windows: `winget install ffmpeg`, then find the camera name:
   `ffmpeg -list_devices true -f dshow -i dummy` (e.g. `"USB Camera"`).
2. Windows: start the streamer (leave running; the loop survives disconnects):

   ```powershell
   while ($true) { ffmpeg -f dshow -video_size 640x480 -framerate 30 `
     -i video="USB Camera" -c:v mjpeg -q:v 6 -f mpjpeg -listen 1 `
     http://0.0.0.0:8090/cam.mjpg; Start-Sleep 1 }
   ```

   Click **Allow** on the firewall prompt.
3. Container:

   ```bash
   ros2 run dofbot_vision stream_camera \
     --ros-args -p url:=http://host.docker.internal:8090/cam.mjpg
   ```

Either way, verify: `ros2 topic hz /image_raw` (~30 Hz) and view with
`ros2 run rqt_image_view rqt_image_view`. If the image is blurry, **twist
the lens barrel** — these cameras are manual-focus; set it at your working
distance (~20–40 cm).

### YOLO detection

```bash
cd <repo>   # so yolov8n.pt resolves
ros2 launch dofbot_vision yolo.launch.py image_topic:=/image_raw model:=yolov8n.pt device:=cpu
```

Watch `/detections` (`ros2 topic echo`) and `/detections/image` (in
rqt_image_view). Show it a cup, bottle, or phone — expect a few FPS on CPU.

---

## 5. Object picking

With the arm stack (section 3), camera, and YOLO all running:

```bash
ros2 launch dofbot_vision pick.launch.py
```

Out of the box this uses a **placeholder calibration** — the arm will run a
full pick sequence at roughly-wrong coordinates. That first run is your
integration test. Then calibrate:

1. **Fix the camera rigidly** viewing the arm's workspace. Any later movement
   invalidates the calibration.
2. Place a small object at **4 widely-spread, non-collinear spots** (corners
   of a rectangle). For each: read the pixel center from
   `ros2 topic echo /detections` (`u=(x1+x2)/2, v=(y1+y2)/2` from
   `bbox_xyxy`), and measure the object's position from the arm's base axis
   in meters (x forward, y left).
3. Compute and install:

   ```bash
   python3 tools/compute_homography.py \
     --image "u1,v1;u2,v2;u3,v3;u4,v4" \
     --base  "x1,y1;x2,y2;x3,y3;x4,y4"
   ```

   Paste the printed YAML into `homography:` in
   `dofbot_ros2_ws/src/dofbot_vision/config/picking.yaml`, then:
   `colcon build --symlink-install --packages-select dofbot_vision` and
   relaunch the picker.
4. First calibrated run: keep `grasp_z: 0.12` — the gripper should stop
   **directly above** the object. Only then lower `grasp_z` stepwise
   (0.08 → 0.05…) until it grasps; tune `gripper_closed` for grip strength.
   Nothing in the software knows where your table is — go gently.

Useful `picking.yaml` knobs: `target_classes` (limit what gets picked),
`pick_once`, `cooldown`, and `place_x/place_y` (enable to make it deposit
objects at a fixed spot — instant pick-and-place demo).

---

## 6. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `ros2: command not found` | shell not sourced → `source /opt/ros/humble/setup.bash && source install/setup.bash` (every new terminal) |
| `ModuleNotFoundError: serial` | `apt install python3-serial` |
| `_ARRAY_API not found` / `KeyError: 16` / `canonicalize_version()` error | pip pin bypassed → see the ⚠️ block in A3 |
| RViz opens but no robot / `robot_description` errors | stale or unsourced build → clean rebuild (`rm -rf build install log`) |
| Controllers stuck `inactive` | plugin load error in `ros2_control_node` output; usually stale build artifacts |
| RViz Execute succeeds, arm doesn't move | driver not running / can't open serial → check driver log; `ros2 topic echo /target_joints` while executing |
| Arm stopped responding (WSL2) | usbipd attachment dropped (replug/sleep) → `ls /dev/ttyUSB*`, re-`attach` |
| IK error `-31` on every pick | move_group missing position-only IK → `ros2 param get /move_group robot_description_kinematics.arm.position_only_ik` must be `True`; if not, rebuild `dofbot_moveit_config` and restart MoveIt |
| Camera attaches (WSL2) but `/image_raw` silent, `dmesg` spams `vhci ... Not yet implemented` | usbipd can't stream webcams, period → use the ffmpeg bridge (section 4) |
| Bridge node: `Connection refused` | ffmpeg not running (it exits when a client disconnects) → use the `while ($true)` loop; check firewall allowed port 8090 |
| Blurry camera image | manual focus → twist the lens barrel |
| Picks consistently offset | camera moved after calibration → recalibrate |
| Arm jumps at startup | arm wasn't posed straight-up before starting the driver (it's open-loop and assumes centered start) |
