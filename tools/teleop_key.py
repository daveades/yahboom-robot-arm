#!/usr/bin/env python3
"""Drive the arm from the keyboard, one keypress at a time.

Two modes, toggled with 'm':

  JOINT      pick a joint (1-5) and nudge it. Direct, no IK, always
             works - the safest way to explore what the arm can do.
  CARTESIAN  nudge the GRASP POINT (between the fingertips) along the
             base frame's x/y/z. Uses the same closed-form IK as the
             chess tools, so what you learn here transfers.

Works against the driver (scripts/driver.sh) or the simulation
(scripts/sim.sh) - both serve the same trajectory actions.

    python3 tools/teleop_key.py            # start in joint mode
    python3 tools/teleop_key.py --cartesian

Each keypress sends ONE motion and waits for it to finish; keys typed
during a motion are discarded rather than queued, so leaning on a key
can never stack up a dozen pending moves.
"""
import argparse
import math
import sys
import termios
import tty

import rclpy

from arm_client import ArmClient, ARM_JOINTS, HOME_POSE

HELP = """
  ---------------- DOFBOT keyboard teleop ----------------
  JOINT mode                CARTESIAN mode
    1..5  select joint        w / s   +x / -x   (forward / back)
    j     joint -= step       a / d   +y / -y   (left / right)
    k     joint += step       q / e   +z / -z   (up / down)

  Both modes
    m     switch mode         o     open gripper
    h     home (straight up)  c     close gripper
    p     print pose          [ / ] gripper by a small step
    + / - step bigger/smaller ?     this help
    Ctrl-C or 'x'  quit
  --------------------------------------------------------
"""


def forward_kinematics(joints, tool_len):
    """Joint angles (rad) -> grasp point (x, y, z) and claw tilt (deg).

    The arm is planar: joint1 yaws the plane, joints 2-4 pitch inside
    it. Each segment points along direction(c) = (-sin c, cos c) in
    (radial, vertical), where c is the cumulative pitch so far - so the
    whole chain is three hops from the shoulder.
    """
    j1, j2, j3, j4 = joints[0], joints[1], joints[2], joints[3]
    rho = zeta = 0.0
    for length, cum in ((ArmClient.L2, j2),
                        (ArmClient.L3, j2 + j3),
                        (tool_len, j2 + j3 + j4)):
        rho += length * -math.sin(cum)
        zeta += length * math.cos(cum)
    tilt = math.degrees(j2 + j3 + j4) + 180.0
    tilt = 180.0 - ((180.0 - tilt) % 360.0)   # wrap into (-180, 180]
    return (rho * math.cos(j1), rho * math.sin(j1),
            zeta + ArmClient.SHOULDER_Z, tilt)


class Teleop:
    def __init__(self, node: ArmClient, args) -> None:
        self.node = node
        self.cartesian = args.cartesian
        self.joint_step = math.radians(args.joint_step)
        self.xyz_step = args.xyz_step
        self.grip_step = args.grip_step
        self.grip_open = args.grip_open
        self.grip_closed = args.grip_closed
        self.selected = 0                      # index into ARM_JOINTS
        self.grip = args.grip_open
        self.joints = node.current_positions()
        self.x, self.y, self.z, _ = forward_kinematics(self.joints,
                                                       node.tool_len)

    # ---------- reporting ----------

    def pose_line(self) -> str:
        x, y, z, tilt = forward_kinematics(self.joints, self.node.tool_len)
        degs = " ".join(f"{math.degrees(v):+6.1f}" for v in self.joints)
        mode = "CARTESIAN" if self.cartesian else "JOINT"
        sel = "" if self.cartesian else f" [joint{self.selected + 1}]"
        step = (f"{self.xyz_step * 1000:.0f}mm" if self.cartesian
                else f"{math.degrees(self.joint_step):.0f}deg")
        return (f"  {mode}{sel} step {step} | joints {degs} | "
                f"grasp x={x:+.3f} y={y:+.3f} z={z:+.3f} tilt {tilt:+.0f}deg "
                f"| grip {self.grip:+.2f}")

    # ---------- motion ----------

    def _send_joints(self) -> None:
        if not self.node.move_joints(list(self.joints)):
            print("  !! motion rejected - restoring last known pose")
            self.joints = self.node.current_positions()

    def _send_cartesian(self) -> None:
        solved = self.node.solve_ik(self.x, self.y, self.z)
        if solved is None:
            print(f"  !! ({self.x:+.3f}, {self.y:+.3f}, {self.z:+.3f}) is "
                  "out of reach - stepping back")
            self.x, self.y, self.z, _ = forward_kinematics(
                self.joints, self.node.tool_len)
            return
        self.joints = solved
        self._send_joints()

    def nudge_joint(self, delta: float) -> None:
        target = self.joints[self.selected] + delta
        if abs(target) > ArmClient.JOINT_LIMIT:
            print(f"  !! joint{self.selected + 1} would exceed its "
                  f"+/-90deg limit")
            return
        self.joints[self.selected] = target
        self._send_joints()

    def nudge_xyz(self, dx: float, dy: float, dz: float) -> None:
        self.x += dx
        self.y += dy
        self.z += dz
        self._send_cartesian()

    def set_grip(self, value: float) -> None:
        self.grip = max(-1.54, min(0.0, value))
        self.node.set_gripper(self.grip)

    def go_home(self) -> None:
        self.joints = list(HOME_POSE)
        self._send_joints()
        self.x, self.y, self.z, _ = forward_kinematics(self.joints,
                                                       self.node.tool_len)

    # ---------- key dispatch ----------

    def handle(self, key: str) -> bool:
        """Act on one key. Returns False to quit."""
        if key in ("x", "\x03"):
            return False
        if key == "?":
            print(HELP)
        elif key == "p":
            print(self.pose_line())
        elif key == "m":
            self.cartesian = not self.cartesian
            self.joints = self.node.current_positions()
            self.x, self.y, self.z, _ = forward_kinematics(
                self.joints, self.node.tool_len)
            print(self.pose_line())
        elif key == "h":
            print("  homing (arm straight up) ...")
            self.go_home()
        elif key == "o":
            self.set_grip(self.grip_open)
        elif key == "c":
            self.set_grip(self.grip_closed)
        elif key == "[":
            self.set_grip(self.grip - self.grip_step)
        elif key == "]":
            self.set_grip(self.grip + self.grip_step)
        elif key in ("+", "="):
            if self.cartesian:
                self.xyz_step = min(0.05, self.xyz_step * 2)
            else:
                self.joint_step = min(math.radians(30), self.joint_step * 2)
        elif key in ("-", "_"):
            if self.cartesian:
                self.xyz_step = max(0.001, self.xyz_step / 2)
            else:
                self.joint_step = max(math.radians(0.5), self.joint_step / 2)
        elif not self.cartesian and key in "12345":
            self.selected = int(key) - 1
        elif not self.cartesian and key == "j":
            self.nudge_joint(-self.joint_step)
        elif not self.cartesian and key == "k":
            self.nudge_joint(+self.joint_step)
        elif self.cartesian and key in "wsadqe":
            s = self.xyz_step
            self.nudge_xyz(*{"w": (+s, 0, 0), "s": (-s, 0, 0),
                             "a": (0, +s, 0), "d": (0, -s, 0),
                             "q": (0, 0, +s), "e": (0, 0, -s)}[key])
        else:
            return True
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cartesian", action="store_true",
                        help="start in cartesian mode (default: joint mode)")
    parser.add_argument("--joint-step", type=float, default=5.0,
                        help="joint-mode step in degrees (default 5)")
    parser.add_argument("--xyz-step", type=float, default=0.01,
                        help="cartesian step in meters (default 0.01)")
    parser.add_argument("--grip-step", type=float, default=0.1,
                        help="gripper fine step (default 0.1)")
    parser.add_argument("--grip-open", type=float, default=-1.1,
                        help="grip_joint position for open (default -1.1)")
    parser.add_argument("--grip-closed", type=float, default=-1.42,
                        help="grip_joint position for closed (default -1.42)")
    parser.add_argument("--move-time", type=float, default=0.8,
                        help="minimum seconds per nudge (default 0.8)")
    parser.add_argument("--max-speed", type=float, default=0.5,
                        help="peak joint speed in rad/s (default 0.5)")
    args = parser.parse_args()

    if not sys.stdin.isatty():
        sys.exit("teleop needs an interactive terminal (a real tty).")

    rclpy.init()
    node = ArmClient("teleop_key", move_time=args.move_time,
                     max_speed=args.max_speed)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        if not node.wait_ready():
            return 1
        print("Settling 4s (lets any driver startup correction finish) ...")
        node.settle(4.0)
        teleop = Teleop(node, args)
        print(HELP)
        print(teleop.pose_line())
        tty.setcbreak(fd)
        while True:
            key = sys.stdin.read(1)
            if not teleop.handle(key):
                break
            # Motions block. Anything typed while the arm was moving is
            # stale intent - drop it so a held key can't stack up moves.
            termios.tcflush(fd, termios.TCIFLUSH)
            print(teleop.pose_line())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        print("\nTeleop stopped. The arm holds its last pose.")
        node.destroy_node()
        # Ctrl-C inside an rclpy call may already have torn the context
        # down; a second shutdown raises RCLError.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
