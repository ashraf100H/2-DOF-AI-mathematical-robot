"""State-transition checks using the original FK and canned angle predictions."""

import ast
import json
from pathlib import Path
import unittest

import numpy as np

from pickup_simulation import simulate_pickup


def load_original_robot():
    path = Path(__file__).resolve().parents[1] / "forward_kinematics.ipynb"
    notebook = json.loads(path.read_text(encoding="utf-8"))
    definitions = []
    for cell in notebook["cells"]:
        source = "".join(cell.get("source", []))
        if cell["cell_type"] == "code" and "class RobotArm2DOF" in source:
            definitions.extend(node for node in ast.parse(source).body
                               if isinstance(node, ast.ClassDef) and node.name == "RobotArm2DOF")
    if len(definitions) != 1:
        raise AssertionError("Expected the single original robot class.")
    namespace = {"np": np}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), "exec"), namespace)
    return namespace["RobotArm2DOF"](10, 10)


class CannedPredictor:
    def __init__(self, outputs):
        self.outputs = list(outputs)
        self.targets = []

    def __call__(self, x, y):
        self.targets.append(np.array([x, y]))
        if not self.outputs:
            raise AssertionError("The simulation made an unexpected extra prediction.")
        return self.outputs.pop(0)


class PickupSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_original_robot()

    def hand(self, angles):
        return np.asarray(self.robot.forward_kinematics(*angles)[2], dtype=float)

    def successful_fixture(self):
        # These existing FK configurations have the same X and different Y.
        # Derive fixture positions from the original robot, without new FK math.
        approach = (-30.0, 90.0)
        lifted = (30.0, -90.0)
        low, high = self.hand(approach), self.hand(lifted)
        if high[1] < low[1]:
            approach, lifted = lifted, approach
            low, high = high, low
        box = low + np.array([0.2, 0.1])
        return approach, lifted, box, float(high[1] - low[1])

    def test_success_keeps_offset_and_frames_independent(self):
        approach, lifted, box, lift = self.successful_fixture()
        predictor = CannedPredictor([approach, lifted])
        result = simulate_pickup(self.robot, predictor, box, lift_cm=lift, frames_per_move=7)
        summary, frames = result["summary"], result["frames"]
        self.assertEqual(summary["status"], "success")
        self.assertTrue(summary["success"])
        self.assertTrue(summary["attached"])
        self.assertEqual(len(predictor.targets), 2)
        np.testing.assert_allclose(predictor.targets[1], self.hand(approach) + [0, lift])
        np.testing.assert_allclose(summary["final_box_cm"], box + [0, lift], atol=1e-9)
        self.assertAlmostEqual(summary["actual_lift_cm"], lift)
        grasp_index = next(i for i, frame in enumerate(frames) if frame["phase"] == "grasped")
        np.testing.assert_array_equal(frames[grasp_index]["box_xy"], frames[grasp_index - 1]["box_xy"])
        for frame in frames:
            np.testing.assert_allclose(frame["hand_xy"], self.hand(frame["angles"]), atol=1e-10)
            self.assertTrue(np.isfinite(frame["angles"]).all())
            self.assertTrue((frame["angles"] >= [-90, -120]).all())
            self.assertTrue((frame["angles"] <= [180, 120]).all())
            if frame["attached"]:
                np.testing.assert_allclose(frame["box_xy"] - frame["hand_xy"], [0.2, 0.1], atol=1e-9)
            else:
                np.testing.assert_array_equal(frame["box_xy"], box)
        json.dumps(summary, allow_nan=False)
        # Rendering may read frames repeatedly or out of order without simulation.
        before = [(f["angles"].copy(), f["box_xy"].copy()) for f in frames]
        for frame in frames[::-1] + frames:
            _ = frame["hand_xy"].tolist(), frame["box_xy"].tolist()
        for frame, (angles, position) in zip(frames, before):
            np.testing.assert_array_equal(frame["angles"], angles)
            np.testing.assert_array_equal(frame["box_xy"], position)
        for index in range(len(frames) - 1):
            for field in ("angles", "hand_xy", "box_xy"):
                self.assertFalse(np.shares_memory(frames[index][field], frames[index + 1][field]))

    def test_invalid_approach_never_moves(self):
        for invalid in ([np.nan, 0], [0, np.inf], [181, 0], [0, -121], [0], [[0, 0]]):
            with self.subTest(prediction=invalid):
                predictor = CannedPredictor([invalid])
                result = simulate_pickup(self.robot, predictor, [12, 8])
                self.assertEqual(result["summary"]["status"], "approach_invalid")
                self.assertIsNone(result["summary"]["approach_error_cm"])
                self.assertFalse(result["summary"]["attached"])
                self.assertEqual(len(predictor.targets), 1)
                for frame in result["frames"]:
                    np.testing.assert_array_equal(frame["angles"], [90, 0])
                    np.testing.assert_array_equal(frame["box_xy"], [12, 8])
                json.dumps(result["summary"], allow_nan=False)

    def test_missed_pickup_moves_arm_but_not_box(self):
        predictor = CannedPredictor([(0, 0)])
        result = simulate_pickup(self.robot, predictor, [12, 8], frames_per_move=4)
        self.assertEqual(result["summary"]["status"], "pickup_missed")
        self.assertGreater(result["summary"]["approach_error_cm"], 0.5)
        self.assertEqual(len(predictor.targets), 1)
        self.assertTrue(any(frame["phase"] == "approach" for frame in result["frames"]))
        for frame in result["frames"]:
            np.testing.assert_array_equal(frame["box_xy"], [12, 8])
            self.assertFalse(frame["attached"])
        np.testing.assert_array_equal(result["frames"][-1]["angles"], [0, 0])

    def test_invalid_or_inaccurate_lift_holds_attached_box(self):
        approach, _, box, _ = self.successful_fixture()
        # (0, 0) would raise this box but miss horizontally; it checks the full
        # Cartesian tolerance independently of the separate no-rise condition.
        for invalid_lift in ([181, 0], [0, np.nan], [0], approach, (0, 0)):
            with self.subTest(prediction=invalid_lift):
                predictor = CannedPredictor([approach, invalid_lift])
                result = simulate_pickup(self.robot, predictor, box, frames_per_move=3)
                summary = result["summary"]
                self.assertEqual(summary["status"], "lift_rejected")
                self.assertFalse(summary["success"])
                self.assertTrue(summary["attached"])
                self.assertAlmostEqual(summary["actual_lift_cm"], 0)
                self.assertAlmostEqual(summary["final_box_error_cm"], 2)
                self.assertFalse(any(f["phase"] == "lift" for f in result["frames"]))
                np.testing.assert_array_equal(result["frames"][-1]["angles"], approach)
                np.testing.assert_array_equal(result["frames"][-1]["box_xy"], box)
                self.assertEqual(len(predictor.targets), 2)

    def test_exact_grasp_tolerance_is_inclusive(self):
        approach = (0, 90)
        box = self.hand(approach) + [0.5, 0]
        predictor = CannedPredictor([approach, [181, 0]])
        result = simulate_pickup(self.robot, predictor, box, grasp_tolerance_cm=0.5)
        self.assertEqual(result["summary"]["approach_error_cm"], 0.5)
        self.assertEqual(result["summary"]["status"], "lift_rejected")
        self.assertTrue(result["summary"]["attached"])

    def test_no_rise_is_rejected_even_inside_large_tolerance(self):
        approach, _, box, _ = self.successful_fixture()
        predictor = CannedPredictor([approach, approach])
        result = simulate_pickup(self.robot, predictor, box, lift_cm=0.1, grasp_tolerance_cm=0.5)
        self.assertLess(result["summary"]["lift_prediction_box_error_cm"], 0.5)
        self.assertEqual(result["summary"]["status"], "lift_rejected")

    def test_invalid_configuration_raises_before_prediction(self):
        cases = [{"box_xy": [np.nan, 0]}, {"box_xy": [0]},
                 {"start_angles": [181, 0]}, {"start_angles": [0, np.inf]},
                 {"lift_cm": 0}, {"lift_cm": -2}, {"lift_cm": np.inf},
                 {"grasp_tolerance_cm": -0.5}, {"grasp_tolerance_cm": np.nan},
                 {"frames_per_move": 0}, {"frames_per_move": 2.5},
                 {"frames_per_move": True}]
        for changes in cases:
            with self.subTest(changes=changes):
                predictor = CannedPredictor([])
                arguments = {"box_xy": [12, 8], **changes}
                with self.assertRaises(ValueError):
                    simulate_pickup(self.robot, predictor, **arguments)
                self.assertEqual(predictor.targets, [])


if __name__ == "__main__":
    unittest.main()
