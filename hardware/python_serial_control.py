"""Frozen ANN -> validated robot angles -> USB serial -> UNO servo controller.

Offline: python hardware/python_serial_control.py --target 10 10
Hardware: python hardware/python_serial_control.py --port COM3
The first command never opens a serial port. Read the calibration guide before
using hardware mode. DONE means commanded trajectory complete, not measured pose.
"""

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import threading
import time

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from trajectory_planner import (Workspace, PlannerSettings, finite_pair,
                                within_limits, robot_points, nearby_jump)


@dataclass
class DeviceProfile:
    calibrated: bool
    lower: np.ndarray
    upper: np.ndarray
    home: np.ndarray
    speed: float

    @classmethod
    def parse(cls, line):
        fields = line.split()
        if len(fields) != 10 or fields[:2] != ["INFO", "1"] or fields[2] not in ("0", "1"):
            raise ValueError("Unexpected controller INFO/version.")
        values = np.asarray(fields[3:], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError("Invalid controller calibration values.")
        lower, upper = values[[0, 2]], values[[1, 3]]
        home, speed = values[4:6], float(values[6])
        if (not within_limits(lower) or not within_limits(upper) or np.any(lower >= upper)
                or np.any(home < lower) or np.any(home > upper) or not 0 < speed <= 30):
            raise ValueError("Controller limits/home/speed do not match this robot.")
        return cls(fields[2] == "1", lower, upper, home, speed)

    def check(self, angles):
        if np.any(angles < self.lower) or np.any(angles > self.upper):
            raise ValueError(f"Outside calibrated hardware limits: {self.lower} to {self.upper} deg.")


def validate_angles(robot, angles, profile=None, current=None):
    angles = finite_pair(angles)
    if not within_limits(angles):
        raise ValueError("Predicted angles exceed the original robot joint limits.")
    if profile:
        profile.check(angles)
    if current is not None and nearby_jump(robot, current, angles, PlannerSettings()):
        raise ValueError("Nearby endpoint requires a large joint/branch change; command rejected.")
    return angles


def evaluate_target(robot, predict, workspace, target, profile=None, current=None):
    target = finite_pair(target)
    if not workspace.contains(target):
        raise ValueError("Target is outside the sampled workspace screen.")
    angles = validate_angles(robot, predict(float(target[0]), float(target[1])), profile, current)
    # Check FK/error at the exact 0.001-degree values used by the serial protocol.
    angles = validate_angles(robot, np.round(angles, 3), profile, current)
    achieved = robot_points(robot, angles)[2]
    error = float(np.hypot(*(achieved - target)))
    if error > 0.5:
        raise ValueError(f"ANN endpoint error {error:.3f} cm exceeds 0.5 cm; no command sent.")
    return angles, achieved, error


class SerialController:
    """One outstanding move, checked acknowledgements, and a 0.4-second heartbeat.

    The worker only writes PING; all reads belong to this main thread. A shared
    write lock prevents interleaved commands. Link loss/timeout ends the session;
    the firmware independently detaches if heartbeats stop for 1.5 seconds.
    """

    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()
        self.quit_heartbeat = threading.Event()
        self.heartbeat_thread = None
        self.heartbeat_error = None
        self.profile = None
        self.current = None
        self.sequence = 0
        self.armed = False

    def send(self, message):
        with self.lock:
            packet = (message + "\n").encode("ascii")
            if self.port.write(packet) != len(packet):
                raise ConnectionError("Incomplete serial write.")

    def receive(self, expected, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.heartbeat_error:
                raise ConnectionError("Serial heartbeat failed.") from self.heartbeat_error
            data = self.port.readline(128)
            if not data:
                continue
            if not data.endswith(b"\n"):
                raise ConnectionError("Partial or oversized controller reply.")
            reply = data.decode("ascii").strip()
            if reply.startswith("ERR") or reply == "READY 1":
                raise ConnectionError(f"Controller fault or reset: {reply}")
            if reply == expected or reply.startswith(expected + " "):
                return reply
            raise ConnectionError(f"Unexpected controller reply: {reply}")
        raise TimeoutError(f"Controller did not reply with {expected}.")

    def identify(self):
        self.send("INFO")
        self.profile = DeviceProfile.parse(self.receive("INFO"))
        return self.profile

    def _heartbeat(self):
        while not self.quit_heartbeat.wait(0.4):
            try:
                self.send("PING")
            except Exception as error:
                self.heartbeat_error = error
                return

    def arm(self):
        if self.armed:
            raise ValueError("Already armed; do not reset the assumed position during a session.")
        if self.profile is None or not self.profile.calibrated:
            raise ValueError("Calibrate the arm and set CALIBRATION_VERIFIED in the sketch first.")
        self.send("ARM")
        self.receive("ARMED")
        self.armed = True
        self.current = self.profile.home.copy()  # Assumed/aligned, NOT encoder feedback.
        self.heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def move(self, angles):
        if not self.armed:
            raise ValueError("Controller is not armed.")
        angles = finite_pair(angles)
        self.profile.check(angles)
        if not within_limits(angles):
            raise ValueError("Robot joint limit exceeded.")
        # Validate the exact rounded values sent to the controller as well.
        sent = np.round(angles, 3)
        self.profile.check(sent)
        self.sequence += 1
        if self.sequence > 65535:
            raise ValueError("Sequence exhausted; stop and begin a newly aligned session.")
        self.send(f"MOVE {self.sequence} {sent[0]:.3f} {sent[1]:.3f}")
        acknowledgement = self.receive("OK")
        if acknowledgement != f"OK {self.sequence}":
            raise ConnectionError("Acknowledgement sequence mismatch.")
        duration = max(0.2, 1.5 * float(np.max(np.abs(sent - self.current))) / self.profile.speed)
        fields = self.receive("DONE", duration + 3).split()
        if len(fields) != 4 or fields[1] != str(self.sequence):
            raise ConnectionError("Completion sequence mismatch.")
        reported = finite_pair(fields[2:])
        if not np.allclose(reported, sent, rtol=0, atol=0.002):
            raise ConnectionError("Controller completed at unexpected commanded angles.")
        self.current = reported
        return reported.copy()

    def close(self):
        self.quit_heartbeat.set()
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=1)
        try:
            self.send("STOP")
            # A pending DONE/ERR may precede STOPPED; bounded draining on exit.
            deadline = time.monotonic() + 1
            confirmed = False
            while time.monotonic() < deadline:
                if self.port.readline(128).strip() == b"STOPPED":
                    confirmed = True
                    break
            if not confirmed:
                print("STOP was not acknowledged. Use the servo power switch.", file=sys.stderr)
        except Exception:
            print("STOP could not be confirmed. Use the servo power switch.", file=sys.stderr)
        finally:
            self.armed = False
            self.port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial port, e.g. COM3; omission keeps output offline")
    parser.add_argument("--target", nargs=2, type=float, metavar=("X", "Y"), default=(10, 10))
    args = parser.parse_args()
    from robot_pipeline import load_robot_pipeline
    robot, predict = load_robot_pipeline(ROOT)
    workspace = Workspace.from_repository(ROOT)
    if not args.port:
        angles, achieved, error = evaluate_target(robot, predict, workspace, args.target)
        print(f"OFFLINE: angles {angles}; FK endpoint {achieved} cm; ANN error {error:.3f} cm.")
        print("No serial port opened. Physical calibration limits and actual position are unverified.")
        return

    import serial
    port = serial.Serial(args.port, 115200, timeout=0.2, write_timeout=0.5)
    controller = SerialController(port)
    try:
        # Opening a UNO port normally resets it; no motion is enabled on reboot.
        time.sleep(2)
        port.reset_input_buffer()
        profile = controller.identify()
        if not profile.calibrated:
            raise ValueError("Firmware reports CALIBRATION_REQUIRED. Follow hardware/calibration_checklist.md.")
        print(f"Hardware limits {profile.lower} to {profile.upper} degrees; home {profile.home}.")
        print("Clear the sweep area. With servo power OFF, align the arm to HOME without forcing gears.")
        print("Power the servos, then type ARM. Startup alignment is manual; there is no encoder feedback.")
        if input("> ").strip() != "ARM":
            return
        controller.arm()
        print("Enter x y (cm), or 'angles a b' for calibration only. 'quit'/'stop' or Ctrl+C detaches.")
        while True:
            text = input("Target> ").strip()
            if text.lower() in ("quit", "stop", "exit"):
                break
            try:
                fields = text.split()
                if len(fields) == 3 and fields[0].lower() == "angles":
                    angles = validate_angles(robot, fields[1:], profile, controller.current)
                    print(f"CALIBRATION command; model endpoint {robot_points(robot, angles)[2]} cm.")
                else:
                    angles, achieved, error = evaluate_target(robot, predict, workspace, fields,
                                                               profile, controller.current)
                    print(f"ANN angles {angles}; model endpoint {achieved} cm; error {error:.3f} cm.")
            except ValueError as error:
                print(f"Rejected: {error}")
                continue
            # Serial exceptions propagate out, STOP the session, and require a
            # fresh alignment. Do not continue using an uncertain robot state.
            sent = controller.move(angles)
            print(f"Commanded motion complete at {sent}. Measure the actual pointer on your grid.")
    finally:
        controller.close()


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("Session stopped.")
    except Exception as error:
        print(f"Stopped: {error}", file=sys.stderr)
        sys.exit(1)
