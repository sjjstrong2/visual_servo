from __future__ import annotations
import numpy as np
import math
from typing import Optional
import logging

def make_homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Compose 4x4 homogeneous transform from 3x3 R and 3x1 t."""
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t.ravel()
    return T

def clamp_vector(v: np.ndarray, max_norm: float) -> np.ndarray:
    """Clamp vector by Euclidean norm (if max_norm <=0, no clamp)."""
    if max_norm <= 0:
        return v
    norm = float(np.linalg.norm(v))
    if norm <= max_norm:
        return v
    return (v / norm) * max_norm

def rotation_matrix_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    """Convert 3x3 rotation matrix to roll, pitch, yaw (ZYX convention)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0.0
    return x, y, z

def build_marker_object_points(marker_size_mm: float) -> np.ndarray:
    """Return 4 object points of square marker in marker frame (z=0), units mm."""
    s = float(marker_size_mm)
    return np.array([
        [-s/2, -s/2, 0.0],
        [ s/2, -s/2, 0.0],
        [ s/2,  s/2, 0.0],
        [-s/2,  s/2, 0.0]
    ], dtype=float)

def load_transform(path: str) -> Optional[np.ndarray]:
    """Load 4x4 transform from .npy and validate shape."""
    try:
        M = np.load(path)
        M = np.asarray(M, dtype=float)
        if M.shape != (4,4):
            logging.error("Transform file %s shape %s != (4,4)", path, M.shape)
            return None
        return M
    except Exception:
        logging.exception("Failed to load transform from %s", path)
        return None

def cam_marker_to_base_marker(T_cam_marker: np.ndarray,
                              T_base_tcp: np.ndarray,
                              T_tcp_cam: np.ndarray) -> np.ndarray:
    """
    Compute T_base_marker = T_base_tcp @ T_tcp_cam @ T_cam_marker
    All matrices are 4x4 numpy arrays. Units are preserved (e.g., mm).
    """
    return T_base_tcp @ T_tcp_cam @ T_cam_marker
