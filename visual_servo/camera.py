from __future__ import annotations
import cv2
import numpy as np
import logging
from typing import Optional, Tuple
from .utils import make_homogeneous, build_marker_object_points

try:
    import cv2.aruco as aruco
except Exception:
    # cv2.aruco available through cv2 usually
    aruco = cv2.aruco

class CameraAruco:
    """Camera wrapper that detects an ArUco marker and returns a 4x4 pose (camera->marker)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.aruco_dict = aruco.getPredefinedDictionary(self.cfg.aruco_dict)
        self.cap = cv2.VideoCapture(self.cfg.camera_id)
        self._apply_capture_settings()
        self.K, self.dist = self._load_camera_params(self.cfg.camera_matrix_path)
        self.obj_points = build_marker_object_points(self.cfg.marker_size_mm)

    def _apply_capture_settings(self):
        # Set some safe defaults; user can change via OS/camera config
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        # Optionally set resolution, exposure here if needed

    @staticmethod
    def _load_camera_params(path: str) -> Tuple[np.ndarray, np.ndarray]:
        try:
            with np.load(path) as X:
                K = X["camera_matrix"]
                dist = X["dist"]
            return K, dist
        except Exception:
            logging.exception("Failed to load camera params from %s; using defaults", path)
            return np.eye(3), np.zeros(5)

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        ret, frame = self.cap.read()
        if not ret:
            logging.error("Camera read failed")
            return False, None
        return True, frame

    def detect(self, frame_gray: np.ndarray):
        corners, ids, _ = aruco.detectMarkers(frame_gray, self.aruco_dict, cameraMatrix=self.K, distCoeff=self.dist)
        return corners, ids

    def estimate_pose(self, corners, idx: int = 0) -> Optional[np.ndarray]:
        """
        SolvePnP to get camera->marker transform (4x4). Returns None if fails.
        The returned transform uses the same units as marker_size_mm (mm).
        """
        if corners is None or len(corners) == 0:
            return None
        try:
            marker_corners = corners[idx].reshape(-1, 2).astype(float)
            ok, rvec, tvec = cv2.solvePnP(self.obj_points, marker_corners, self.K, self.dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                logging.debug("solvePnP returned not ok")
                return None
            R, _ = cv2.Rodrigues(rvec)
            T = make_homogeneous(R, tvec)
            return T
        except Exception:
            logging.exception("estimate_pose error")
            return None

    def draw_marker_overlay(self, frame: np.ndarray, corners, ids):
        if corners is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
        # additional overlays can be added here

    def release(self):
        self.cap.release()
