"""Hardware bridge checks with the original FK and a fake serial transport.

These do not connect to motors or claim to measure actual servo position.
"""
import unittest
from unittest.mock import patch

import numpy as np

from test_pickup_simulation import load_original_robot, CannedPredictor
from python_serial_control import (DeviceProfile, SerialController,
                                             evaluate_target, validate_angles)
from trajectory_planner import Workspace
from pathlib import Path

INFO = "INFO 1 1 0.000 120.000 0.000 110.000 90.000 0.000 15.000"


class FakeSerial:
    def __init__(self):
        self.writes, self.replies = [], []
        self.closed = False
        self.move_replies = None

    def write(self, packet):
        self.writes.append(packet)
        message = packet.decode().strip()
        if message == "INFO": self.replies.append((INFO + "\n").encode())
        elif message == "ARM": self.replies.append(b"ARMED\n")
        elif message == "STOP": self.replies.append(b"STOPPED\n")
        elif message.startswith("MOVE "):
            _, sequence, first, second = message.split()
            self.replies.extend(self.move_replies if self.move_replies is not None else
                                [f"OK {sequence}\n".encode(), f"DONE {sequence} {first} {second}\n".encode()])
        return len(packet)

    def readline(self, size):
        return self.replies.pop(0) if self.replies else b""

    def close(self): self.closed = True


class HardwareBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_original_robot()
        cls.workspace = Workspace.from_repository(Path(__file__).resolve().parents[1])

    def test_controller_profile_is_validated(self):
        profile = DeviceProfile.parse(INFO)
        np.testing.assert_array_equal(profile.lower, [0, 0])
        np.testing.assert_array_equal(profile.upper, [120, 110])
        for text in ["INFO 2 1 0 120 0 110 90 0 15", "INFO 1 1 nan 120 0 110 90 0 15",
                     "INFO 1 1 130 120 0 110 90 0 15", "INFO 1 1 0 190 0 110 90 0 15",
                     "INFO 1 1 0 120 0 110 130 0 15", "INFO 1 1 0 120 0 110 90 0 0",
                     "INFO 1 1 0 120 0 110 90 0 31", INFO + " extra"]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                DeviceProfile.parse(text)

    def test_valid_ann_uses_original_fk(self):
        predictor = CannedPredictor([[0, 90]])
        angles, hand, error = evaluate_target(self.robot, predictor, self.workspace, [10, 10],
                                              DeviceProfile.parse(INFO), [90, 0])
        np.testing.assert_array_equal(angles, [0, 90])
        np.testing.assert_array_equal(hand, [10, 10])
        self.assertEqual(error, 0)
        self.assertEqual(len(predictor.targets), 1)

    def test_bad_input_and_unreachable_do_not_call_ann(self):
        for target in [["bad", 10], [np.nan, 10], [np.inf, 1], [0], [21, 0], [0, 0]]:
            predictor = CannedPredictor([])
            with self.subTest(target=target), self.assertRaises(ValueError):
                evaluate_target(self.robot, predictor, self.workspace, target)
            self.assertEqual(len(predictor.targets), 0)

    def test_bad_ann_limits_and_error_rejected(self):
        for angles in [[181, 0], [0, -121], [np.nan, 0], [0, 0]]:
            with self.subTest(angles=angles), self.assertRaises(ValueError):
                evaluate_target(self.robot, CannedPredictor([angles]), self.workspace, [10, 10])

    def test_hardware_limits_and_branch_jump_are_not_clipped(self):
        with self.assertRaises(ValueError):
            validate_angles(self.robot, [-10, 90], DeviceProfile.parse(INFO))
        with self.assertRaises(ValueError):
            validate_angles(self.robot, [0, 90], current=[90, -90])
        with self.assertRaises(ValueError):
            validate_angles(self.robot, [120.0001, 80], DeviceProfile.parse(INFO))

    def test_fk_error_uses_exact_serial_rounding(self):
        q = [0.506237864, 90.17552185]
        angles, hand, error = evaluate_target(self.robot, CannedPredictor([q]), self.workspace, [10, 10])
        np.testing.assert_array_equal(angles, [0.506, 90.176])
        expected = self.robot.forward_kinematics(*angles)[2]
        np.testing.assert_array_equal(hand, expected)
        self.assertAlmostEqual(error, np.linalg.norm(hand - [10, 10]))

    def test_unverified_controller_cannot_arm(self):
        port = FakeSerial(); link = SerialController(port)
        link.profile = DeviceProfile.parse(INFO.replace("INFO 1 1", "INFO 1 0"))
        with self.assertRaises(ValueError): link.arm()
        self.assertNotIn(b"ARM\n", port.writes)

    def test_handshake_move_and_close(self):
        port = FakeSerial(); link = SerialController(port)
        try:
            link.identify(); link.arm()
            with self.assertRaises(ValueError): link.arm()
            np.testing.assert_array_equal(link.move([0.506, 90.176]), [0.506, 90.176])
            self.assertIn(b"MOVE 1 0.506 90.176\n", port.writes)
            with self.assertRaises(ValueError): link.move([-1, 90])
            self.assertEqual(sum(p.startswith(b"MOVE ") for p in port.writes), 1)
        finally: link.close()
        self.assertTrue(port.closed)
        self.assertEqual(port.writes[-1], b"STOP\n")

    def test_bad_acknowledgement_or_completion_does_not_update_state(self):
        responses = [[b"OK 99\n"], [b"OK 1\n", b"DONE 2 0 90\n"],
                     [b"OK 1\n", b"DONE 1 nan 90\n"], [b"OK 1\n", b"DONE 1 0 80\n"],
                     [b"READY 1\n"], [b"ERR LIMIT\n"], [b"OK 1"]]
        for replies in responses:
            with self.subTest(replies=replies):
                port = FakeSerial(); link = SerialController(port)
                try:
                    link.identify(); link.arm(); port.move_replies = replies
                    with self.assertRaises((ValueError, ConnectionError)):
                        link.move([0, 90])
                    np.testing.assert_array_equal(link.current, [90, 0])
                finally: link.close()
                self.assertIn(b"STOP\n", port.writes)

    def test_missing_reply_times_out(self):
        link = SerialController(FakeSerial())
        with self.assertRaises(TimeoutError): link.receive("OK", timeout=.002)

    def test_heartbeat_failure_propagates(self):
        link = SerialController(FakeSerial())
        link.heartbeat_error = OSError("USB removed")
        with self.assertRaises(ConnectionError): link.receive("DONE")

    def test_short_write_is_detected(self):
        port = FakeSerial(); link = SerialController(port)
        with patch.object(port, "write", return_value=1), self.assertRaises(ConnectionError):
            link.send("ARM")


if __name__ == "__main__": unittest.main()
