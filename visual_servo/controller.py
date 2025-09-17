# ur_visual_servo/controller.py
from __future__ import annotations
import numpy as np
import math
import logging
from typing import Optional, Tuple
from .utils import clamp_vector

# OpenCV is required for Rodrigues; it's already a dependency of the project.
import cv2


class PBVSController:
    """
    PBVS controller (improved).

    Features:
    - Uses cv2.Rodrigues on R_err to compute rotation vector (axis * angle) robustly.
    - Falls back to trace/sin or eigenvector method for angle ~ pi if necessary.
    - Optional PD control (Kp = cfg.linear_gain/angular_gain, Kd passed to constructor).
    - Optional exponential smoothing (alpha filter) on the output velocities.
    - Utility to convert base-frame velocities to tool-frame velocities.
    - Utility to map cartesian 6D velocity to joint velocities via Damped Least Squares.

    Inputs/Outputs:
    - compute_velocity(current_T, desired_T, dt=None) -> (v_lin (3,), v_ang (3,))
      current_T, desired_T: 4x4 homogeneous transforms (translation in mm).
      v_lin returned in m/s; v_ang in rad/s.
    """

    def __init__(self, cfg, kd: float = 0.0, alpha: float = 0.0):
        """
        cfg: config object (expects linear_gain, angular_gain, max_linear_speed, max_angular_speed, control_rate_hz)
        kd: derivative gain (same units structure as Kp; set 0 to disable D-term)
        alpha: smoothing factor in [0,1); 0 = no smoothing; typical small values ~0.1
        """
        self.cfg = cfg
        self.kd = float(kd)
        self.alpha = float(alpha) if (alpha is not None) else 0.0
        # internal previous-state for derivative & filtering
        self._prev_e_t = np.zeros(3, dtype=float)
        self._prev_e_r = np.zeros(3, dtype=float)
        self._prev_v_lin = np.zeros(3, dtype=float)
        self._prev_v_ang = np.zeros(3, dtype=float)
        # small tolerance for numeric checks
        self._eps = 1e-9

    def compute_velocity(self,
                         current_T: np.ndarray,
                         desired_T: np.ndarray,
                         dt: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute commanded Cartesian velocities.
        - current_T, desired_T: 4x4 transforms in same frame (translation units: mm)
        - dt: elapsed time in seconds (if None, inferred from cfg.control_rate_hz)
        Returns:
        - v_lin: (3,) numpy array (m/s)
        - v_ang: (3,) numpy array (rad/s)
        """
        assert current_T.shape == (4, 4) and desired_T.shape == (4, 4)

        # estimate dt
        if dt is None:
            try:
                dt = 1.0 / float(max(1e-6, getattr(self.cfg, "control_rate_hz", 20.0)))
            except Exception:
                dt = 1.0 / 20.0

        # ---------- translation error (m) ----------
        t_cur_m = current_T[:3, 3] / 1000.0
        t_des_m = desired_T[:3, 3] / 1000.0
        e_t = t_des_m - t_cur_m  # (m)

        # ---------- rotation error (axis-angle) ----------
        R_cur = current_T[:3, :3]
        R_des = desired_T[:3, :3]
        R_err = R_des @ R_cur.T

        # Try robust route: use cv2.Rodrigues(R_err) to get rotation vector (axis*angle)
        e_r = None
        try:
            rvec, _ = cv2.Rodrigues(R_err)  # returns (3,1)
            e_r = rvec.ravel()
            # small-norm numerical cleanup
            if np.linalg.norm(e_r) < 1e-12:
                e_r = np.zeros(3, dtype=float)
        except Exception:
            logging.debug("cv2.Rodrigues failed on R_err; falling back to robust trace-based computation.")
            # fallback: trace-based formula (classic)
            cos_theta = (np.trace(R_err) - 1.0) / 2.0
            cos_theta = float(max(-1.0, min(1.0, cos_theta)))
            angle = math.acos(cos_theta)
            if abs(angle) < 1e-12:
                e_r = np.zeros(3, dtype=float)
            else:
                sin_theta = math.sin(angle)
                # handle near-pi case separately because of numeric instability
                if abs(math.pi - angle) < 1e-3 or abs(sin_theta) < 1e-6:
                    # robust axis extraction for angle near pi: find eigenvector with eigenvalue ~1
                    try:
                        vals, vecs = np.linalg.eig(R_err)
                        # find eigenvalue closest to 1
                        idx = int(np.argmin(np.abs(vals - 1.0)))
                        axis = np.real(vecs[:, idx])
                        norm = np.linalg.norm(axis)
                        if norm < 1e-8:
                            axis = np.zeros(3, dtype=float)
                        else:
                            axis = axis / norm
                        e_r = axis * angle
                    except Exception:
                        # as last resort, use antisymmetric part / (2*sin)
                        rx = (R_err[2, 1] - R_err[1, 2]) / (2.0 * max(1e-8, sin_theta))
                        ry = (R_err[0, 2] - R_err[2, 0]) / (2.0 * max(1e-8, sin_theta))
                        rz = (R_err[1, 0] - R_err[0, 1]) / (2.0 * max(1e-8, sin_theta))
                        axis = np.array([rx, ry, rz], dtype=float)
                        e_r = axis * angle
                else:
                    rx = (R_err[2, 1] - R_err[1, 2]) / (2.0 * sin_theta)
                    ry = (R_err[0, 2] - R_err[2, 0]) / (2.0 * sin_theta)
                    rz = (R_err[1, 0] - R_err[0, 1]) / (2.0 * sin_theta)
                    axis = np.array([rx, ry, rz], dtype=float)
                    e_r = axis * angle

        # ---------- PD term (optional) ----------
        # P part:
        v_lin = float(getattr(self.cfg, "linear_gain", 1.0)) * e_t
        v_ang = float(getattr(self.cfg, "angular_gain", 1.0)) * e_r

        # D part (if kd > 0)
        if self.kd and self.kd != 0.0:
            d_e_t = (e_t - self._prev_e_t) / max(dt, 1e-8)
            d_e_r = (e_r - self._prev_e_r) / max(dt, 1e-8)
            v_lin = v_lin + self.kd * d_e_t
            v_ang = v_ang + self.kd * d_e_r

        # ---------- clamp (norm-based) ----------
        v_lin = clamp_vector(v_lin, float(getattr(self.cfg, "max_linear_speed", 0.05)))
        v_ang = clamp_vector(v_ang, float(getattr(self.cfg, "max_angular_speed", 0.3)))

        # ---------- smoothing (alpha filter) ----------
        if 0.0 < self.alpha < 1.0:
            v_lin = self.alpha * self._prev_v_lin + (1.0 - self.alpha) * v_lin
            v_ang = self.alpha * self._prev_v_ang + (1.0 - self.alpha) * v_ang

        # ---------- update internal state ----------
        self._prev_e_t = e_t.copy()
        self._prev_e_r = e_r.copy()
        self._prev_v_lin = v_lin.copy()
        self._prev_v_ang = v_ang.copy()

        return v_lin, v_ang

    # ----------------- utilities -----------------
    @staticmethod
    def base_to_tool_velocity(v_lin_base: np.ndarray,
                              v_ang_base: np.ndarray,
                              T_base_tcp: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert velocities expressed in base frame to tool (TCP) frame.
        - v_lin_base, v_ang_base: (3,) numpy arrays (m/s, rad/s)
        - T_base_tcp: 4x4 transform from base->tcp (translation units don't matter here)
        Returns:
        - v_lin_tool, v_ang_tool: both (3,) arrays in tool frame
        Formula:
            v_tool = R_base_tcp^T * v_base
            w_tool = R_base_tcp^T * w_base
        """
        R = T_base_tcp[:3, :3]
        v_tool = R.T @ v_lin_base
        w_tool = R.T @ v_ang_base
        return v_tool, w_tool

    @staticmethod
    def map_cartesian_to_joint_velocities(v_lin: np.ndarray,
                                          v_ang: np.ndarray,
                                          J: np.ndarray,
                                          damping: float = 1e-2) -> np.ndarray:
        """
        Map 6D Cartesian velocity [v_lin; v_ang] to joint velocities q_dot using
        Damped Least Squares (Tikhonov regularization).

        Inputs:
        - v_lin: (3,) m/s
        - v_ang: (3,) rad/s
        - J: Jacobian of shape (6, n) (linear+angular rows) mapping q_dot -> [v_lin; v_ang]
        - damping: scalar >=0 (lambda). Larger -> more damping (stable near singularities)

        Returns:
        - q_dot: (n,) numpy array of joint velocities (rad/s)
        Formula:
          q_dot = J^T * (J J^T + lambda^2 I)^(-1) * v
        """
        v = np.concatenate([v_lin, v_ang], axis=0).astype(float)  # (6,)
        J = np.asarray(J, dtype=float)
        if J.ndim != 2 or J.shape[0] != 6:
            raise ValueError("J must be shape (6, n)")
        # compute (JJ^T + lambda^2 I)
        JJt = J @ J.T  # (6,6)
        reg = (damping ** 2) * np.eye(JJt.shape[0], dtype=float)
        A = JJt + reg
        # solve A * y = v for y
        try:
            y = np.linalg.solve(A, v)
        except np.linalg.LinAlgError:
            # fallback to pseudo-inverse with SVD
            y = np.linalg.lstsq(A, v, rcond=None)[0]
        q_dot = J.T @ y  # (n,)
        return q_dot
