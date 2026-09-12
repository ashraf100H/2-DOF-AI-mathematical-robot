"""A small geometric pickup simulation using an existing robot and ANN helper.

The gripper is an ideal point. Joint interpolation makes the motion visible;
it is not Cartesian path planning and does not check collisions or dynamics.
"""

import numpy as np


JOINT_MIN_DEGREES = np.array([-90.0, -120.0])
JOINT_MAX_DEGREES = np.array([180.0, 120.0])


def _finite_pair(value, name):
    try:
        pair = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must contain two finite numbers.") from error
    if pair.shape != (2,) or not np.isfinite(pair).all():
        raise ValueError(f"{name} must contain two finite numbers.")
    return pair.copy()


def _finite_scalar(value, name, *, allow_zero=False):
    if isinstance(value, (bool, np.bool_)) or np.ndim(value) != 0:
        raise ValueError(f"{name} must be a finite scalar.")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite scalar.") from error
    if not np.isfinite(number) or number < 0 or (number == 0 and not allow_zero):
        condition = "nonnegative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {condition}.")
    return number


def _within_joint_limits(angles):
    return bool(((angles >= JOINT_MIN_DEGREES) & (angles <= JOINT_MAX_DEGREES)).all())


def _hand_position(robot, angles):
    # All robot geometry comes from the supplied validated implementation.
    _, _, hand_xy = robot.forward_kinematics(float(angles[0]), float(angles[1]))
    return _finite_pair(hand_xy, "Forward-kinematics endpoint")


def _predict_valid_angles(predict_angles, target):
    try:
        angles = _finite_pair(
            predict_angles(float(target[0]), float(target[1])), "Predicted angles"
        )
    except (TypeError, ValueError, OverflowError) as error:
        return None, str(error)
    if not _within_joint_limits(angles):
        return None, "Predicted angles are outside the robot's joint limits."
    return angles, None


def simulate_pickup(
    robot,
    predict_angles,
    box_xy,
    start_angles=(90.0, 0.0),
    lift_cm=2.0,
    grasp_tolerance_cm=0.5,
    frames_per_move=30,
):
    """Return precomputed frames and a JSON-serializable pickup summary.

    ``predict_angles(x, y)`` must return the frozen ANN's angles in degrees.
    No prediction is clipped, corrected, or replaced by another IK solver.
    Invalid predictions stop motion. A missed grasp leaves the box unchanged;
    a rejected lift leaves it attached at the last accepted pose.
    """
    initial_box = _finite_pair(box_xy, "Box position")
    current_angles = _finite_pair(start_angles, "Start angles")
    if not _within_joint_limits(current_angles):
        raise ValueError("Start angles must satisfy the robot's joint limits.")
    lift_cm = _finite_scalar(lift_cm, "Lift distance")
    tolerance = _finite_scalar(grasp_tolerance_cm, "Grasp tolerance", allow_zero=True)
    if (isinstance(frames_per_move, (bool, np.bool_))
            or not isinstance(frames_per_move, (int, np.integer))
            or frames_per_move < 1):
        raise ValueError("frames_per_move must be a positive integer.")
    if not callable(predict_angles):
        raise ValueError("predict_angles must be callable.")
    if not callable(getattr(robot, "forward_kinematics", None)):
        raise ValueError("robot must provide forward_kinematics.")

    with np.errstate(over="ignore", invalid="ignore"):
        desired_box = initial_box + np.array([0.0, lift_cm])
    desired_box = _finite_pair(desired_box, "Desired final box position")
    current_hand = _hand_position(robot, current_angles)
    current_box = initial_box.copy()
    attached = False
    offset = None
    approach_error = None
    lift_target = None
    lift_prediction_error = None
    frames = []

    def add_frame(phase):
        # Each frame owns its values: render/replay reads state without changing it.
        frames.append({"phase": phase, "angles": current_angles.copy(),
                       "hand_xy": current_hand.copy(), "box_xy": current_box.copy(),
                       "attached": attached})

    def move_to(target_angles, phase):
        nonlocal current_angles, current_hand, current_box
        move_start = current_angles.copy()
        for step in range(1, frames_per_move + 1):
            fraction = step / frames_per_move
            current_angles = (target_angles.copy() if step == frames_per_move else
                              move_start + fraction * (target_angles - move_start))
            current_hand = _hand_position(robot, current_angles)
            if attached:
                current_box = current_hand + offset
            add_frame(phase)

    def finish(status, message):
        add_frame(status)
        return {"frames": frames, "summary": {
            "status": status, "success": status == "success",
            "box_target_cm": initial_box.tolist(),
            "desired_box_final_cm": desired_box.tolist(),
            "final_box_cm": current_box.tolist(),
            "approach_error_cm": approach_error,
            "final_box_error_cm": float(np.linalg.norm(current_box - desired_box)),
            "grasp_tolerance_cm": tolerance, "requested_lift_cm": lift_cm,
            "actual_lift_cm": float(current_box[1] - initial_box[1]),
            "attached": attached, "message": message,
            "attachment_offset_cm": None if offset is None else offset.tolist(),
            "lift_target_cm": None if lift_target is None else lift_target.tolist(),
            "lift_prediction_box_error_cm": lift_prediction_error,
        }}

    add_frame("ready")
    approach_angles, reason = _predict_valid_angles(predict_angles, initial_box)
    if approach_angles is None:
        return finish("approach_invalid", f"Approach rejected before motion: {reason}")

    # A valid configuration may still miss badly: display that approach faithfully.
    approach_hand = _hand_position(robot, approach_angles)
    approach_error = float(np.linalg.norm(approach_hand - initial_box))
    move_to(approach_angles, "approach")
    if approach_error > tolerance:
        return finish("pickup_missed", "The hand missed the grasp tolerance; the box stayed put.")

    attached = True
    offset = initial_box - current_hand
    add_frame("grasped")

    # Preserve the attachment offset. Raising the achieved hand by the requested
    # amount asks for the same box displacement without snapping its center.
    lift_target = current_hand + np.array([0.0, lift_cm])
    lift_angles, reason = _predict_valid_angles(predict_angles, lift_target)
    if lift_angles is None:
        return finish("lift_rejected", f"Lift rejected; holding the box at the grasp pose: {reason}")
    predicted_box = _hand_position(robot, lift_angles) + offset
    lift_prediction_error = float(np.linalg.norm(predicted_box - desired_box))
    if lift_prediction_error > tolerance or predicted_box[1] <= initial_box[1]:
        return finish("lift_rejected", "The predicted lift does not meet the box-position tolerance "
                      "or raise the box; holding the grasp pose.")

    move_to(lift_angles, "lift")
    return finish("success", "The box attached without snapping and reached the lift tolerance.")
