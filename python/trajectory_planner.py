"""Small, conservative 2D planner. IK chooses the goal; planning checks the route.

All robot coordinates come from the supplied original RobotArm2DOF. The ANN is
the only source of target/waypoint joint angles; rejected predictions are never
clipped or replaced with analytical IK or a dataset lookup.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from pickup_simulation import JOINT_MIN_DEGREES, JOINT_MAX_DEGREES


@dataclass(frozen=True)
class Rectangle:
    xmin: float
    xmax: float
    ymin: float
    ymax: float


@dataclass(frozen=True)
class Scene:
    # Centimeters. The box is a selectable target, not a grasp/contact model.
    table: Rectangle = Rectangle(-18.0, -12.0, -12.0, 6.0)
    floor_y: float = -12.0
    box_xy: tuple = (-14.0, 6.5)
    box_side: float = 1.0


@dataclass(frozen=True)
class PlannerSettings:
    target_tolerance_cm: float = 0.5
    clearance_cm: float = 0.15
    collision_step_deg: float = 0.5
    waypoint_spacing_cm: float = 2.0
    clearance_heights_cm: tuple = (14.0, 17.0)
    nearby_distance_cm: float = 3.0
    nearby_jump_deg: float = 60.0
    waypoint_jump_deg: float = 120.0
    overhead_waypoint_cm: tuple = (0.0, 15.0)
    joint_speed_deg_s: float = 45.0


@dataclass
class PlanResult:
    status: str
    message: str
    target: np.ndarray | None = None
    prediction: np.ndarray | None = None
    error_cm: float | None = None
    mode: str = "--"
    nodes: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    waypoints: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    direct_reason: str = ""
    attempts: list = field(default_factory=list)

    @property
    def accepted(self):
        return self.status == "ready"


def finite_pair(value):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("Enter two finite numbers in centimeters.") from error
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError("Enter two finite numbers in centimeters.")
    return result.copy()


def within_limits(angles):
    return bool(np.isfinite(angles).all() and
                ((angles >= JOINT_MIN_DEGREES) & (angles <= JOINT_MAX_DEGREES)).all())


def robot_points(robot, angles):
    return np.asarray(robot.forward_kinematics(float(angles[0]), float(angles[1])), dtype=float)


class Workspace:
    """A reachability *screen*, using raw FK positions, never their angle labels.

    A 0.27 cm neighborhood covers the half-cell displacement bound of the 1-degree
    grid (20*pi/360 + 10*pi/360 = 0.262 cm). The screen is approximate at joint-limit
    boundaries; the ANN's limit and FK-error checks are still mandatory.
    """

    def __init__(self, positions):
        self.positions = np.asarray(positions, dtype=float)
        radii = np.hypot(self.positions[:, 0], self.positions[:, 1])
        self.minimum_radius = float(radii.min())
        self.maximum_radius = float(radii.max())

    @classmethod
    def from_repository(cls, root):
        return cls(np.loadtxt(Path(root) / "data/robot_configurations.csv",
                              delimiter=",", skiprows=1, usecols=(2, 3)))

    def contains(self, point):
        radius = float(np.hypot(*point))
        if not self.minimum_radius - 1e-8 <= radius <= self.maximum_radius + 1e-8:
            return False
        distance = np.hypot(*(self.positions - point).T)
        return bool(distance.min() <= 0.27)


def segment_intersects_rectangle(start, end, rectangle, padding=0.0):
    """Slab intersection: retain the segment's t interval inside both axes.

    Touching an edge/corner counts as collision, including a zero-length segment.
    Padding expands the forbidden rectangle, providing a conservative clearance.
    """
    lower = (rectangle.xmin - padding, rectangle.ymin - padding)
    upper = (rectangle.xmax + padding, rectangle.ymax + padding)
    enter, leave = 0.0, 1.0
    for axis in range(2):
        delta = end[axis] - start[axis]
        if abs(delta) < 1e-12:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
        else:
            first = (lower[axis] - start[axis]) / delta
            last = (upper[axis] - start[axis]) / delta
            enter = max(enter, min(first, last))
            leave = min(leave, max(first, last))
            if enter > leave:
                return False
    return True


def collision_reason(robot, angles, scene, padding=0.15):
    points = robot_points(robot, angles)
    if np.min(points[:, 1]) <= scene.floor_y + padding:
        return "Robot touches the floor clearance."
    for index in range(2):
        if segment_intersects_rectangle(points[index], points[index + 1], scene.table, padding):
            return f"Link {index + 1} intersects the table clearance."
    return ""


def segment_is_clear(robot, start, end, scene, settings):
    """Check the *whole* joint segment, not just its endpoints.

    At each interval midpoint, inflate obstacles by the maximum possible link
    displacement over half that interval: (L1+L2)*|dtheta1| + L2*|dtheta2|.
    Angles here are radians. This upper bound covers every point of both links,
    so an obstacle cannot slip between the sampled configurations. Rejections
    can be conservative near a surface. Smooth time scaling follows this same
    configuration segment, so the geometric check applies at every time.
    """
    if not within_limits(start) or not within_limits(end):
        return False, "Joint limits would be exceeded."
    count = max(1, int(np.ceil(np.max(np.abs(end - start)) / settings.collision_step_deg)))
    def check_interval(first, last, depth=0):
        midpoint = (first + last) / 2
        half_delta = np.radians(np.abs(last - first) / 2)
        sweep_bound = (robot.L1 + robot.L2) * half_delta[0] + robot.L2 * half_delta[1]
        padding = settings.clearance_cm + sweep_bound + 1e-8  # Rounded FK coordinates.
        reason = collision_reason(robot, midpoint, scene, padding)
        if not reason:
            return True, ""
        # Refine ambiguous near-obstacle intervals, instead of rejecting a safe
        # path just because its conservative sweep bound was too large.
        if depth >= 7 or collision_reason(robot, midpoint, scene, settings.clearance_cm):
            return False, reason
        clear, reason = check_interval(first, midpoint, depth + 1)
        return check_interval(midpoint, last, depth + 1) if clear else (False, reason)

    for index in range(count):
        clear, reason = check_interval(start + index / count * (end - start),
                                       start + (index + 1) / count * (end - start))
        if not clear:
            return False, reason
    return True, ""


def nearby_jump(robot, start, end, settings):
    start, end = np.asarray(start), np.asarray(end)
    distance = np.linalg.norm(robot_points(robot, start)[2] - robot_points(robot, end)[2])
    return (distance <= settings.nearby_distance_cm and
            np.max(np.abs(end - start)) > settings.nearby_jump_deg)


def _cartesian_samples(corners, spacing):
    points = []
    for start, end in zip(corners[:-1], corners[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(end - start) / spacing)))
        points.extend(end.copy() if step == count else start + step / count * (end - start)
                      for step in range(1, count + 1))
    return points


def plan_motion(robot, predict_angles, workspace, current_angles, target_xy,
                scene=Scene(), settings=PlannerSettings()):
    """Try a checked direct joint path, then three finite clearance-route attempts.

    This is an intentionally incomplete planner. Failure means this policy did
    not find a route, not that no physical route exists. Return no partial motion
    on failure. Long direct moves can be legitimate; the branch guard compares
    nearby endpoints, while *every* small Cartesian waypoint gets a jump check.
    """
    result = PlanResult("invalid_input", "")
    try:
        target = finite_pair(target_xy)
        current = finite_pair(current_angles)
    except ValueError as error:
        result.message = str(error)
        return result
    result.target = target
    if not within_limits(current):
        result.message = "Current configuration is invalid; reset the scene."
        return result
    reason = collision_reason(robot, current, scene, settings.clearance_cm)
    if reason:
        result.status, result.message = "collision", "Current pose: " + reason
        return result

    def candidate(point):
        if not workspace.contains(point):
            return None, None, "unreachable", "Outside the joint-limited workspace screen."
        if (point[1] <= scene.floor_y + settings.clearance_cm or
                segment_intersects_rectangle(point, point, scene.table, settings.clearance_cm)):
            return None, None, "collision", "Target is inside the table/floor clearance."
        try:
            angles = finite_pair(predict_angles(float(point[0]), float(point[1])))
        except Exception as error:
            return None, None, "invalid_prediction", f"ANN prediction failed: {error}"
        if not within_limits(angles):
            return angles, None, "invalid_prediction", "ANN angles violate joint limits; no clipping applied."
        error = float(np.linalg.norm(robot_points(robot, angles)[2] - point))
        if error > settings.target_tolerance_cm:
            return angles, error, "ann_error", f"ANN endpoint error {error:.3f} cm exceeds {settings.target_tolerance_cm:.2f} cm."
        reason = collision_reason(robot, angles, scene, settings.clearance_cm)
        if reason:
            return angles, error, "collision", "Predicted goal pose: " + reason
        return angles, error, "ready", ""

    goal, result.error_cm, result.status, result.message = candidate(target)
    result.prediction = goal
    if result.status != "ready":
        return result
    direct_clear, reason = segment_is_clear(robot, current, goal, scene, settings)
    if nearby_jump(robot, current, goal, settings):
        direct_clear, reason = False, "Nearby endpoints require a joint change above the branch-jump limit."
    result.direct_reason = reason
    if direct_clear:
        result.mode, result.message = "DIRECT PATH", "Direct joint path passed collision and continuity checks."
        result.nodes = np.asarray([current, goal])
        return result

    hand = robot_points(robot, current)[2]
    cache = {tuple(target): (goal, result.error_cm, "ready", "")}
    routes = []
    for height in settings.clearance_heights_cm:
        # Up/over/approach is a candidate, never an assumption of safety. The arm
        # can collide even while its hand is above the table; check both links.
        corners = np.asarray([hand, [hand[0], height], [target[0], height], target])
        routes.append((f"Clearance y={height:g} cm",
                       _cartesian_samples(corners, settings.waypoint_spacing_cm)))
    # A coarse overhead detour avoids forcing straight Cartesian legs through
    # the central unreachable region. These are checked JOINT paths between two
    # ANN goals, not a promise that the hand follows straight Cartesian lines.
    routes.append(("Overhead waypoint", [np.asarray(settings.overhead_waypoint_cm), target]))
    for name, points in routes:
        nodes = [current]
        failure = ""
        for point in points:
            key = tuple(point)
            if key not in cache:
                cache[key] = candidate(point)
            angles, _, status, message = cache[key]
            if status != "ready":
                failure = f"Waypoint ({point[0]:.1f}, {point[1]:.1f}): {message}"
                break
            previous_request = hand if len(nodes) == 1 else points[len(nodes) - 2]
            close_request = np.linalg.norm(point - previous_request) <= settings.nearby_distance_cm
            jump_limit = settings.nearby_jump_deg if close_request else settings.waypoint_jump_deg
            if (np.max(np.abs(angles - nodes[-1])) > jump_limit or
                    nearby_jump(robot, nodes[-1], angles, settings)):
                failure = "Waypoint ANN branch jump exceeds the joint continuity limit."
                break
            clear, message = segment_is_clear(robot, nodes[-1], angles, scene, settings)
            if not clear:
                failure = "Waypoint segment: " + message
                break
            nodes.append(angles)
        result.attempts.append(f"{name}: {failure or 'accepted'}")
        if not failure:
            result.mode, result.nodes = "WAYPOINT PATH", np.asarray(nodes)
            result.waypoints = np.asarray(points)
            result.message = f"Direct path rejected. {name} accepted after checking every segment."
            return result
    result.status, result.mode = "no_path", "NO SAFE PATH"
    result.message = "No route accepted. " + result.attempts[-1]
    return result


class JointTrajectory:
    """Speed-limited cubic easing along accepted joint-space segments.

    Smoothstep's peak slope is 1.5, so duration >= 1.5*max_joint_change/speed.
    Each waypoint has zero velocity. There is no angle wrapping or teleporting.
    UI playback advances with a fixed timestep and uses this same time function.
    """

    def __init__(self, nodes, speed_deg_s=45.0):
        self.nodes = np.asarray(nodes, dtype=float).copy()
        if (self.nodes.ndim != 2 or self.nodes.shape[1] != 2 or len(self.nodes) < 2
                or not np.isfinite(self.nodes).all() or not np.isfinite(speed_deg_s)
                or speed_deg_s <= 0):
            raise ValueError("Trajectory requires finite joint pairs and a positive speed.")
        changes = np.max(np.abs(np.diff(self.nodes, axis=0)), axis=1)
        self.durations = np.maximum(0.2, 1.5 * changes / speed_deg_s)
        self.ends = np.cumsum(self.durations)
        self.duration = float(self.ends[-1])

    def sample(self, elapsed):
        if elapsed <= 0:
            return self.nodes[0].copy()
        if elapsed >= self.duration:
            return self.nodes[-1].copy()
        index = int(np.searchsorted(self.ends, elapsed, side="right"))
        previous_end = 0.0 if index == 0 else self.ends[index - 1]
        fraction = (elapsed - previous_end) / self.durations[index]
        eased = fraction * fraction * (3 - 2 * fraction)
        return self.nodes[index] + eased * (self.nodes[index + 1] - self.nodes[index])
