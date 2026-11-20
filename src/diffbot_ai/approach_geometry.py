"""Pure geometry helpers. These functions do not command a robot."""

import math


def deproject_rectified(u, v, depth_m, fx, fy, cx, cy):
    """Deproject aligned rectified depth to optical coordinates (x right, y down, z forward)."""
    values = (u, v, depth_m, fx, fy, cx, cy)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('All camera values must be finite')
    if depth_m <= 0 or fx <= 0 or fy <= 0:
        raise ValueError('Depth and focal lengths must be positive')
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)


def approach_pose(robot_xy, target_xy, stop_distance_m):
    """Return map-frame (x, y, yaw), or None when already within standoff.

    The standoff is measured from the robot base origin to the supplied target
    point. This helper does not check obstacles, footprint, or goal feasibility.
    """
    rx, ry = robot_xy
    tx, ty = target_xy
    if not all(math.isfinite(value) for value in (rx, ry, tx, ty, stop_distance_m)):
        raise ValueError('Coordinates and distance must be finite')
    if stop_distance_m <= 0:
        raise ValueError('Stop distance must be positive')
    dx, dy = tx - rx, ty - ry
    distance = math.hypot(dx, dy)
    if distance <= stop_distance_m:
        return None
    yaw = math.atan2(dy, dx)
    return (tx - stop_distance_m * dx / distance,
            ty - stop_distance_m * dy / distance, yaw)
