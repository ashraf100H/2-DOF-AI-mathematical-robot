"""Optional integration check with saved TensorFlow artifacts (no training).

Run from the repository root: python python/check_frozen_planner.py
Writes only simulation/step7_checks.json. Lightweight unit tests run separately.
"""

from hashlib import sha256
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
os.environ.setdefault("TF_ENABLE_ONEDNN_OPTS", "0")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

import numpy as np

from robot_pipeline import load_robot_pipeline
from pickup_simulation import simulate_pickup
from trajectory_planner import (Scene, Workspace, JointTrajectory, plan_motion,
                                collision_reason, within_limits)


def main():
    protected = [*(ROOT / "notebooks").glob("*.ipynb"), *(ROOT / "data").glob("*"),
                 *(ROOT / "models").glob("*"), *(ROOT / "evaluation").glob("*"),
                 ROOT / "python/robot_pipeline.py", ROOT / "python/pickup_simulation.py",
                 ROOT / "simulation/pickup_results.json"]
    hashes = {p.relative_to(ROOT).as_posix(): sha256(p.read_bytes()).hexdigest()
              for p in protected if p.is_file()}
    robot, predict = load_robot_pipeline(ROOT)
    workspace = Workspace.from_repository(ROOT)
    scene = Scene()
    results = []

    def check(name, target, expected, start=(90, 0), mode=None):
        result = plan_motion(robot, predict, workspace, start, target)
        assert result.status == expected, (name, result.message)
        if mode:
            assert result.mode == mode, (name, result.mode)
        record = {"case": name, "target_cm": list(target), "start_angles_deg": list(start),
                  "status": result.status, "mode": result.mode, "error_cm": result.error_cm,
                  "predicted_angles_deg": None if result.prediction is None else result.prediction.tolist(),
                  "direct_reason": result.direct_reason, "attempts": result.attempts,
                  "message": result.message}
        if result.accepted:
            motion = JointTrajectory(result.nodes)
            for t in np.linspace(0, motion.duration, 2001):
                angles = motion.sample(t)
                assert within_limits(angles)
                assert not collision_reason(robot, angles, scene), (name, t)
            record["duration_seconds"] = motion.duration
            record["nodes_deg"] = result.nodes.tolist()
            record["waypoints_cm"] = result.waypoints.tolist()
        else:
            assert len(result.nodes) == 0
        results.append(record)
        print(f"{name}: {result.status}, {result.mode}, error={result.error_cm}")
        return result

    check("direct", (10, 10), "ready", mode="DIRECT PATH")
    start = check("detour_start", (-10, 4), "ready", mode="DIRECT PATH")
    check("waypoint", (-14, 14), "ready", start.prediction, "WAYPOINT PATH")
    check("box", scene.box_xy, "ready", mode="DIRECT PATH")
    check("ann_error", (10, -10), "ann_error")
    check("unreachable", (21, 0), "unreachable")
    check("collision", (-15, 0), "collision")
    prior = json.loads((ROOT / "simulation/pickup_results.json").read_text())
    regression = {}
    for name in ("first_attempt", "reference_demo", "known_failure_demo"):
        expected = prior[name]
        actual = simulate_pickup(robot, predict, expected["box_target_cm"])["summary"]
        assert actual["status"] == expected["status"]
        np.testing.assert_allclose(actual["final_box_cm"], expected["final_box_cm"], atol=1e-6)
        np.testing.assert_allclose(actual["approach_error_cm"], expected["approach_error_cm"], atol=1e-5)
        regression[name] = actual["status"]
    for name, digest in hashes.items():
        assert sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
    report = {"scene_cm": {"table": vars(scene.table), "floor_y": scene.floor_y, "box_xy": scene.box_xy},
              "examples": results, "step6_regression": regression, "source_sha256": hashes,
              "selection": "Selected engineering examples, not a model accuracy estimate. The overhead (0,15) waypoint and outer-left table were selected during integration; rejected route attempts remain visible.",
              "model_retrained": False, "ik_policy_changed": False}
    (ROOT / "simulation/step7_checks.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("7 frozen-model planner checks and all 3 Step 6 regressions passed; protected inputs unchanged.")


if __name__ == "__main__":
    main()
