"""Planner geometry/state tests: original FK, explicit canned ANN responses.

No TensorFlow import or substitute IK solver is required for these unit tests.
Frozen-model integration examples are recorded separately in simulation/.
"""

from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from test_pickup_simulation import load_original_robot, CannedPredictor
from trajectory_planner import (Scene, Rectangle, PlannerSettings, Workspace,
                                JointTrajectory, plan_motion, robot_points,
                                segment_intersects_rectangle, segment_is_clear,
                                collision_reason, nearby_jump, within_limits)


class TrajectoryPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = load_original_robot()
        cls.workspace = Workspace.from_repository(Path(__file__).resolve().parents[1])
        cls.scene = Scene()
        cls.settings = PlannerSettings()

    def plan(self, predictor, target=(10, 10), start=(90, 0), **kwargs):
        return plan_motion(self.robot, predictor, self.workspace, start, target, **kwargs)

    def test_clear_direct_path_accepted(self):
        predictor = CannedPredictor([(0, 90)])
        result = self.plan(predictor)
        self.assertTrue(result.accepted)
        self.assertEqual(result.mode, "DIRECT PATH")
        self.assertEqual(result.error_cm, 0)
        np.testing.assert_array_equal(result.nodes, [[90, 0], [0, 90]])
        self.assertEqual(len(predictor.targets), 1)

    def test_segment_rectangle_edges_and_degenerate_cases(self):
        rect = Rectangle(1, 3, 2, 4)
        cases = [((0, 3), (4, 3), True), ((0, 2), (4, 2), True),
                 ((2, 0), (2, 6), True), ((0, 1), (1, 2), True),
                 ((2, 3), (2, 3), True), ((0, 0), (0, 0), False),
                 ((0, 1), (4, 1), False), ((0, 0), (0, 5), False)]
        for a, b, expected in cases:
            with self.subTest(a=a, b=b):
                self.assertEqual(segment_intersects_rectangle(a, b, rect), expected)

    def test_both_links_and_floor_checked(self):
        for rectangle, expected in [(Rectangle(5, 6, -1, 1), "Link 1"),
                                    (Rectangle(15, 16, -1, 1), "Link 2")]:
            scene = replace(self.scene, table=rectangle)
            self.assertIn(expected, collision_reason(self.robot, [0, 0], scene))
        self.assertIn("floor", collision_reason(self.robot, [-90, 0], self.scene))

    def test_obstacle_between_clear_endpoint_poses_rejected(self):
        # The original model locates a tiny obstacle on the interior of Link 2
        # at 22.5 degrees, away from the start/end and coarse midpoint poses.
        points = robot_points(self.robot, [22.5, 0])
        x, y = (points[1] + points[2]) / 2
        scene = replace(self.scene, table=Rectangle(x-.001, x+.001, y-.001, y+.001))
        settings = replace(self.settings, clearance_cm=0, collision_step_deg=90)
        for angle in ([0, 0], [90, 0], [45, 0]):
            self.assertFalse(collision_reason(self.robot, angle, scene, 0))
        clear, _ = segment_is_clear(self.robot, np.array([0., 0.]), np.array([90., 0.]), scene, settings)
        self.assertFalse(clear)

    def test_waypoint_route_when_direct_path_is_blocked(self):
        # Recorded ANN angles, used only as deterministic test responses. Other
        # waypoint requests intentionally fail; the overhead route must work.
        start = np.array([99.83238983154297, 115.27679443359375])
        goal = np.array([128.0044708251953, 16.130504608154297])
        overhead = np.array([50.3167610168457, 82.0775146484375])

        def predictor(x, y):
            if np.allclose([x, y], [-14, 14]):
                return goal
            if np.allclose([x, y], [0, 15]):
                return overhead
            return [np.nan, np.nan]

        self.assertFalse(segment_is_clear(self.robot, start, goal, self.scene, self.settings)[0])
        result = self.plan(predictor, [-14, 14], start)
        self.assertTrue(result.accepted, result.message)
        self.assertEqual(result.mode, "WAYPOINT PATH")
        np.testing.assert_array_equal(result.nodes, [start, overhead, goal])
        self.assertEqual(len(result.attempts), 3)
        # Independently inspect dense poses along the *returned* eased motion.
        trajectory = JointTrajectory(result.nodes)
        for t in np.linspace(0, trajectory.duration, 2001):
            angles = trajectory.sample(t)
            self.assertTrue(within_limits(angles))
            self.assertFalse(collision_reason(self.robot, angles, self.scene), (t, angles))

    def test_unreachable_targets_rejected_before_ann(self):
        for target in [(21, 0), (0, 0), (7, 0), (-15, -10), (1e300, 0)]:
            with self.subTest(target=target):
                predictor = CannedPredictor([])
                result = self.plan(predictor, target)
                self.assertEqual(result.status, "unreachable")
                self.assertEqual(len(predictor.targets), 0)
                self.assertEqual(len(result.nodes), 0)

    def test_invalid_input_rejected_without_prediction(self):
        for target in [("oops", 10), (np.nan, 10), (0, np.inf), (1,), [[1, 2]]]:
            predictor = CannedPredictor([])
            result = self.plan(predictor, target)
            self.assertEqual(result.status, "invalid_input")
            self.assertEqual(len(predictor.targets), 0)

    def test_invalid_ann_results_rejected_without_clipping(self):
        for prediction in [(181, 0), (0, -121), (np.inf, 0), (0,), [[0, 90]]]:
            with self.subTest(prediction=prediction):
                result = self.plan(CannedPredictor([prediction]))
                self.assertEqual(result.status, "invalid_prediction")
                self.assertEqual(len(result.nodes), 0)

    def test_ann_exception_is_a_reported_failure(self):
        def broken(x, y):
            raise RuntimeError("Model unavailable")
        result = self.plan(broken)
        self.assertEqual(result.status, "invalid_prediction")
        self.assertIn("Model unavailable", result.message)

    def test_excessive_error_is_not_corrected(self):
        result = self.plan(CannedPredictor([(0, 0)]))
        self.assertEqual(result.status, "ann_error")
        self.assertGreater(result.error_cm, self.settings.target_tolerance_cm)
        np.testing.assert_array_equal(result.prediction, [0, 0])
        self.assertEqual(len(result.nodes), 0)

    def test_target_in_obstacle_rejected_before_ann(self):
        predictor = CannedPredictor([])
        result = self.plan(predictor, (-15, 0))
        self.assertEqual(result.status, "collision")
        self.assertEqual(len(predictor.targets), 0)

    def test_nearby_branch_jump_and_failed_replan_return_no_partial_path(self):
        # The two original-FK configurations reach the SAME (10,10) target.
        self.assertTrue(nearby_jump(self.robot, [90, -90], [0, 90], self.settings))
        def predictor(x, y):
            return [0, 90] if np.allclose([x, y], [10, 10]) else [np.nan, 0]
        result = self.plan(predictor, start=(90, -90))
        self.assertEqual(result.status, "no_path")
        self.assertIn("branch", result.direct_reason)
        self.assertEqual(len(result.nodes), 0)

    def test_colliding_goal_configuration_is_rejected(self):
        goal = [165, 0]
        target = robot_points(self.robot, goal)[2]
        self.assertTrue(collision_reason(self.robot, goal, self.scene))
        predictor = CannedPredictor([goal])
        result = self.plan(predictor, target)
        self.assertEqual(result.status, "collision")
        self.assertEqual(len(predictor.targets), 1)
        self.assertEqual(len(result.nodes), 0)

    def test_motion_is_continuous_and_speed_limited(self):
        nodes = np.array([[90., 0.], [0., 90.], [30., 60.]])
        trajectory = JointTrajectory(nodes, speed_deg_s=45)
        dt = .001
        times = np.arange(0, trajectory.duration, dt)
        angles = np.array([trajectory.sample(t) for t in times])
        self.assertLessEqual(np.max(np.abs(np.diff(angles, axis=0))) / dt, 45 + 1e-6)
        np.testing.assert_array_equal(trajectory.sample(0), nodes[0])
        np.testing.assert_array_equal(trajectory.sample(trajectory.duration), nodes[-1])
        self.assertFalse(np.array_equal(trajectory.sample(.1), nodes[-1]))
        for t, node in zip(trajectory.ends, nodes[1:]):
            np.testing.assert_allclose(trajectory.sample(t), node, atol=1e-10)
        # Easing starts and stops at rest, including internal waypoint joins.
        for t in [0, *trajectory.ends]:
            velocity = (trajectory.sample(t+dt) - trajectory.sample(t-dt)) / (2*dt)
            self.assertLess(np.max(np.abs(velocity)), .15)
        snapshot = trajectory.sample(0)
        snapshot[:] = 999
        np.testing.assert_array_equal(trajectory.sample(0), nodes[0])


if __name__ == "__main__":
    unittest.main()
