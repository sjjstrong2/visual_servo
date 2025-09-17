from __future__ import annotations
import numpy as np
import logging
from typing import Optional

# optional (soft) dependencies
try:
    import rtde_control  # type: ignore
    import rtde_receive  # type: ignore
    HAS_RTDE = True
except Exception:
    HAS_RTDE = False

class URAdapter:
    """
    Adapter for UR robot. If rtde_control/rtde_receive are available and ur_host is provided,
    will attempt to use real robot; otherwise operates in simulation (logging) mode.

    Notes:
    - send_cartesian_velocity expects linear in m/s (3,), angular in rad/s (3,), dt seconds.
    - The frame of the provided velocities must match the robot API expectation.
      This code assumes velocities are in the robot base frame by default.
      If your API expects tool-frame velocities, convert before calling send_cartesian_velocity.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.simulate = (cfg.ur_host is None) or (not HAS_RTDE)
        self.rtde_ctl = None
        self.rtde_rcv = None
        if not self.simulate and HAS_RTDE:
            try:
                self.rtde_ctl = rtde_control.RTDEControlInterface(cfg.ur_host)
                self.rtde_rcv = rtde_receive.RTDEReceiveInterface(cfg.ur_host)
                logging.info("Connected to UR via RTDE at %s", cfg.ur_host)
            except Exception:
                logging.exception("Failed to init RTDE control; will simulate.")
                self.simulate = True

    def send_cartesian_velocity(self, linear: np.ndarray, angular: np.ndarray, dt: float):
        """
        Send Cartesian velocity to robot. Default behavior:
        - If simulate: log command.
        - If real robot (rtde): attempt to call speedL(list(linear), list(angular), dt)
          Note: adapt this method to your RTDE API signature (some wrappers expect different args).
        """
        if self.simulate:
            logging.info("[SIM] send_cartesian_velocity linear=%s m/s angular=%s rad/s dt=%.3f",
                         np.round(linear,4).tolist(), np.round(angular,4).tolist(), dt)
            return

        try:
            if hasattr(self.rtde_ctl, "speedL"):
                # Many rtde_control implementations: speedL(lin, ang, time)
                self.rtde_ctl.speedL(list(linear), list(angular), dt)
            else:
                logging.warning("RTDE control interface has no speedL method - adapt URAdapter for your API.")
        except Exception:
            logging.exception("Failed to send velocity to robot via RTDE")

    def get_current_base_tcp(self) -> Optional[np.ndarray]:
        """
        Try to obtain current T_base_tcp (4x4) from robot. Implementation depends on available API.
        Returns None on failure or in simulation.
        NOTE: Many RTDE wrappers return tool pose as [x,y,z,rx,ry,rz] where rx,ry,rz is rotation vector.
        This method attempts to convert such pose to 4x4 matrix. Verify units (m vs mm) for your API.
        """
        if self.simulate or not HAS_RTDE or self.rtde_rcv is None:
            return None
        try:
            pose = self.rtde_rcv.getActualTCPPose()  # typical name; may vary
            # If pose is None -> fail
            if pose is None:
                return None
            arr = np.asarray(pose, dtype=float)
            # common format [x,y,z,rx,ry,rz]
            if arr.size == 6:
                t = arr[:3]
                rvec = arr[3:]
                try:
                    import cv2
                    R, _ = cv2.Rodrigues(rvec)
                    T = np.eye(4, dtype=float)
                    T[:3, :3] = R
                    T[:3, 3] = t
                    return T
                except Exception:
                    logging.exception("Failed to convert rtde pose rotation vector to matrix.")
                    return None
            # if shape 4x4
            if arr.shape == (4,4):
                return arr
            return None
        except Exception:
            logging.exception("Error reading TCP pose from robot")
            return None
