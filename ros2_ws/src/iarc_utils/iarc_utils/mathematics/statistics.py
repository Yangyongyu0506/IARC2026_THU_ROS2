"""
Placeholder
"""

import math
import numpy as np

def stamp2us(stamp) -> int:
    """
    Converts a ROS2 Time or builtin_interfaces.msg.Time object to microseconds.
    """
    return stamp.sec * 1000_000 + stamp.nanosec // 1000

def lerp(x1, y1: np.ndarray, x2, y2: np.ndarray, x) -> np.ndarray:
    """
    Performs linear interpolation to find the corresponding y value for a given x, based on two known points (x1, y1) and (x2, y2).
    """
    if x2 == x1:
        return 0.5 * (y1 + y2)  # If x1 and x2 are the same, return the average of y1 and y2 to avoid division by zero
    return y1 + (y2 - y1) * ((x - x1) / (x2 - x1))

def slerp(x1, q1: np.ndarray, x2, q2: np.ndarray, x) -> np.ndarray:
    """
    Performs spherical interpolation to find the corresponsing quaternion for a given x, based on two known points (x1, q1) and (x2, q2).
    """
    if x2 == x1:
        return 0.5 * (q1 + q2)  # If x1 and x2 are the same, return the average of q1 and q2 to avoid division by zero
    t = (x - x1) / (x2 - x1)
    if np.dot(q1, q2) < 0.0:
        q2 = -q2  # Ensure the shortest path is taken
    theta = math.acos(np.clip(np.dot(q1, q2), -1.0, 1.0))
    if theta < 1e-6:
        return q1  # If the quaternions are very close, return one of them to avoid numerical instability
    sin_theta = math.sin(theta)
    q = (math.sin((1 - t) * theta) / sin_theta) * q1 + (math.sin(t * theta) / sin_theta) * q2
    return q / np.linalg.norm(q) 