from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import yaml
import logging

@dataclass
class Config:
    # marker & camera
    marker_size_mm: float = 95.0
    Intrinsic_Parameters_path: str = "bin/camera_matrix.npz"
    extrinsic_parameters: str = "bin/Extrinsic_parameters.npz"
    aruco_dict: int = 10  # cv2.aruco.DICT_4X4_250 => numeric constant; default 10 typical for 4x4_250
    camera_id: int = 0

    # control
    control_rate_hz: float = 20.0
    linear_gain: float = 0.8
    angular_gain: float = 0.6
    max_linear_speed: float = 0.05    # m/s
    max_angular_speed: float = 0.3    # rad/s
    safety_timeout_s: float = 1.0

    # file paths
    desired_pose_path: str = "bin/desired_pose.npy"
    desired_tvec_path: str = "bin/desired_realworld_tvec.npy"
    desired_euler_path: str = "bin/desired_euler_angles.npy"

    # UR connection
    ur_host: Optional[str] = None
    ur_port: int = 30002

    # hand-eye / base external transforms
    hand_eye_path: Optional[str] = None   # .npy file (4x4) representing T_tcp_cam or T_cam_tcp
    hand_eye_convention: str = "T_tcp_cam"  # "T_tcp_cam" or "T_cam_tcp"
    base_tcp_path: Optional[str] = None   # optional static T_base_tcp .npy

    # desired pose frame
    desired_pose_frame: str = "camera"    # "camera" or "base"

    # GUI / logging
    show_gui: bool = True
    log_level: str = "INFO"

    @classmethod
    def from_yaml(cls, path: str) -> "Config":
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            cfg = cls()
            for k, v in data.items():
                if hasattr(cfg, k):
                    setattr(cfg, k, v)
            return cfg
        except FileNotFoundError:
            logging.warning("Config YAML not found at %s. Using defaults.", path)
            return cls()
        except Exception:
            logging.exception("Failed to load config YAML. Using defaults.")
            return cls()
