# transformations.py

import numpy as np

def rotation_matrix_to_quaternion(R) -> np.ndarray:
    """
    Calculates the quaternion representation of a rotation matrix.
    """
    q = []
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q.append(0.25 / s)
        q.append((R[2, 1] - R[1, 2]) * s)
        q.append((R[0, 2] - R[2, 0]) * s)
        q.append((R[1, 0] - R[0, 1]) * s)
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q.append((R[2, 1] - R[1, 2]) / s)
            q.append(0.25 * s)
            q.append((R[0, 1] + R[1, 0]) / s)
            q.append((R[0, 2] + R[2, 0]) / s)
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q.append((R[0, 2] - R[2, 0]) / s)
            q.append((R[0, 1] + R[1, 0]) / s)
            q.append(0.25 * s)
            q.append((R[1, 2] + R[2, 1]) / s)
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q.append((R[1, 0] - R[0, 1]) / s)
            q.append((R[0, 2] + R[2, 0]) / s)
            q.append((R[1, 2] + R[2, 1]) / s)
            q.append(0.25 * s)
    return np.array(q[1:4].append(q[0]))  # Return in (x, y, z, w) order

def quaternion_to_rotation_matrix(q) -> np.ndarray:
    """
    Calculates the rotation matrix from a quaternion.
    """
    x, y, z, w =q
    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])
    return R

def euler_to_quaternion(roll, pitch, yaw) -> np.ndarray:
    """
    Calculates the quaternion representation of Euler angles (in radians).
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    q = np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ])
    return q

def quaternion_to_euler(q) -> np.ndarray:
    """
    Calculates the Euler angles (in radians) from a quaternion.
    """
    x, y, z, w = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([roll, pitch, yaw])

def get_normal_vector(points) -> np.ndarray:
    """
    Calculate the normal vector of a plane defined by a series of points using Singular Value Decomposition (SVD)
    """
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    _, _, vh = np.linalg.svd(centered_points)
    normal_vector = vh[-1]
    return normal_vector / np.linalg.norm(normal_vector)

def get_xyz_from_points(points) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Calculate the x, y and z coordinate vectors based on predefined points.
    """
    assert points.shape[0] >= 3, "At least 3 points are required to define the coordinate system."
    x0 = points[1] - points[0]
    y0 = points[-1] - points[0]
    x0 /= np.linalg.norm(x0)
    z = get_normal_vector(points)
    z *= np.sign(z * np.cross(x0, y0))
    z /= np.linalg.norm(z)
    y = np.cross(z, x0)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    return x, y, z