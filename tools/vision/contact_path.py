"""Geometry and trajectory validation shared by the contact planner and tests."""
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation


def position(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


def rotation(pose):
    q = pose.orientation
    return Rotation.from_quat([q.x, q.y, q.z, q.w])


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if not np.all(np.isfinite(vector)) or length < 1e-9:
        raise ValueError('Direction must be finite and nonzero')
    return vector / length


def angle_deg(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(unit(a), unit(b)), -1, 1))))


def check_straight_pose(pose, start, end, reference_rotation, max_rotation_deg=10., tolerance=.002):
    delta = end - start
    direction = unit(delta)
    progress = float(np.dot(position(pose) - start, direction))
    cross = np.linalg.norm(position(pose) - start - progress * direction)
    angle = math.degrees((reference_rotation.inv() * rotation(pose)).magnitude())
    if cross > tolerance or progress < -tolerance or progress > np.linalg.norm(delta) + tolerance:
        raise ValueError('Cartesian trajectory leaves the requested segment')
    if angle > max_rotation_deg:
        raise ValueError('Cartesian trajectory changes the selected tool orientation')
    return float(cross), angle


def seconds(duration):
    return duration.sec + duration.nanosec / 1e9


def set_seconds(duration, value):
    ns = round(value * 1e9)
    duration.sec, duration.nanosec = divmod(ns, 1000000000)


def reverse_trajectory(trajectory):
    result = copy.deepcopy(trajectory)
    points = trajectory.joint_trajectory.points
    if len(points) < 2:
        raise ValueError('Cannot reverse an empty Cartesian trajectory')
    end = seconds(points[-1].time_from_start)
    result.joint_trajectory.points = []
    for point in reversed(points):
        new = copy.deepcopy(point)
        set_seconds(new.time_from_start, end - seconds(point.time_from_start))
        new.velocities = [-v for v in point.velocities]
        # Acceleration is unchanged under time reversal.
        result.joint_trajectory.points.append(new)
    # Both ends are rest states. Time parameterization may leave roundoff
    # velocities; ros2_control requires an exactly zero terminal velocity.
    for point in (result.joint_trajectory.points[0], result.joint_trajectory.points[-1]):
        if any(not math.isfinite(v) or abs(v) > 1e-5 + 1e-12 for v in point.velocities):
            raise ValueError(f'Cartesian endpoint must be at rest: {list(point.velocities)}')
        point.velocities = [0.0 for _ in point.velocities]
    return result


def join_trajectories(first, second):
    a, b = first.joint_trajectory, second.joint_trajectory
    if a.joint_names != b.joint_names:
        raise ValueError('Trajectory joint order differs')
    if not a.points or len(b.points) < 2:
        raise ValueError('Empty trajectory segment')
    if np.max(np.abs(np.array(a.points[-1].positions) - b.points[0].positions)) > .001:
        raise ValueError('Trajectory join has a position discontinuity')
    for point in (a.points[-1], b.points[0]):
        if any(not math.isfinite(v) or abs(v) > 1e-5 + 1e-12 for v in point.velocities):
            raise ValueError('Both segments must stop at the join')
    result = copy.deepcopy(first)
    offset = seconds(a.points[-1].time_from_start)
    start_time = seconds(b.points[0].time_from_start)
    for point in b.points[1:]:
        new = copy.deepcopy(point)
        set_seconds(new.time_from_start, offset + seconds(point.time_from_start) - start_time)
        result.joint_trajectory.points.append(new)
    return result
