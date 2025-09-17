# ur_visual_servo/main.py
from __future__ import annotations
import argparse
import logging
import time
import numpy as np
import cv2
from .config import Config
from .camera import CameraAruco
from .controller import PBVSController
from .ur_interface import URAdapter
from .utils import (rotation_matrix_to_euler, load_transform, cam_marker_to_base_marker)

def parse_args():
    p = argparse.ArgumentParser(description="UR Visual Servo (TCP control via Jacobian)")
    p.add_argument("--config", default="config.yaml", help="YAML config file path")
    p.add_argument("--camera-id", type=int, default=None, help="override camera id")
    p.add_argument("--no-gui", dest="show_gui", action="store_false", help="disable GUI")
    p.add_argument("--simulate", action="store_true", help="force simulate mode (no robot)")
    return p.parse_args()

def main():
    args = parse_args()
    cfg = Config.from_yaml(args.config)
    if args.camera_id is not None:
        cfg.camera_id = args.camera_id
    if not args.show_gui:
        cfg.show_gui = False

    logging.basicConfig(level=getattr(logging, cfg.log_level.upper(), logging.INFO),
                        format="[%(asctime)s] %(levelname)s: %(message)s")

    cam = CameraAruco(cfg)
    # enable small kd/alpha if desired; tune to reduce oscillation
    controller = PBVSController(cfg, kd=0.02, alpha=0.05)
    ur = URAdapter(cfg if not args.simulate else Config())  # if simulate flag set, pass default cfg (ur_host None)

    # Load hand-eye (T_tcp_cam or T_cam_tcp) and normalize to internal T_tcp_cam (4x4)
    T_tcp_cam = np.eye(4)
    if cfg.hand_eye_path:
        he = load_transform(cfg.hand_eye_path)
        if he is None:
            logging.warning("Failed to load hand-eye transform from %s. Using identity.", cfg.hand_eye_path)
        else:
            if cfg.hand_eye_convention == "T_tcp_cam":
                T_tcp_cam = he
            elif cfg.hand_eye_convention == "T_cam_tcp":
                T_tcp_cam = np.linalg.inv(he)
            else:
                logging.warning("hand_eye_convention '%s' unknown; using identity.", cfg.hand_eye_convention)

    # Get initial T_base_tcp: try robot API first, then fallback to file, else identity (warn)
    T_base_tcp_robot = ur.get_current_base_tcp()
    if T_base_tcp_robot is not None:
        # We do not assume robot returns mm; commonly RTDE returns meters. We'll treat robot pose as meters and convert to mm for chain.
        # Save both forms: T_base_tcp_m (meters) and T_base_tcp_mm (mm)
        T_base_tcp_m = T_base_tcp_robot.copy()
        T_base_tcp_mm = T_base_tcp_m.copy()
        T_base_tcp_mm[:3, 3] = T_base_tcp_m[:3, 3] * 1000.0
        logging.info("Obtained T_base_tcp from robot (assumed meters). Converted to mm for vision chain.")
    elif cfg.base_tcp_path:
        tb = load_transform(cfg.base_tcp_path)
        if tb is not None:
            # assume file is in meters; if your file uses mm, change here accordingly
            T_base_tcp_m = tb.copy()
            T_base_tcp_mm = T_base_tcp_m.copy()
            T_base_tcp_mm[:3, 3] = T_base_tcp_m[:3, 3] * 1000.0
            logging.info("Loaded T_base_tcp from %s (assumed meters).", cfg.base_tcp_path)
        else:
            logging.warning("Failed to load base_tcp_path %s; using identity.", cfg.base_tcp_path)
            T_base_tcp_m = np.eye(4)
            T_base_tcp_mm = np.eye(4)
    else:
        logging.warning("No T_base_tcp available (robot not connected and no base_tcp_path). Using identity (NOT recommended).")
        T_base_tcp_m = np.eye(4)
        T_base_tcp_mm = np.eye(4)

    # Load desired pose (in camera frame or base frame as configured)
    try:
        desired_T_raw = np.load(cfg.desired_pose_path)
        logging.info("Loaded desired pose from %s", cfg.desired_pose_path)
    except Exception:
        desired_T_raw = np.eye(4)
        logging.info("No desired pose file found; using identity.")

    # Convert desired pose to base frame if it was saved in camera frame.
    if cfg.desired_pose_frame == "camera":
        # desired_T_raw is camera->marker with mm units
        desired_T_base_mm = cam_marker_to_base_marker(desired_T_raw, T_base_tcp_mm, T_tcp_cam)  # mm
    else:
        desired_T_base_mm = desired_T_raw  # assume already in base frame with mm

    last_detection = time.time()
    rate = 1.0 / cfg.control_rate_hz
    logging.info("Starting main loop at %.1f Hz (show_gui=%s)", cfg.control_rate_hz, cfg.show_gui)

    current_T_cam = None
    current_T_base_mm = None

    try:
        while True:
            loop_start = time.time()
            ok, frame = cam.read()
            if not ok:
                logging.error("Camera read failed - exiting main loop.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = cam.detect(gray)

            detected_and_valid = False
            if corners is not None and len(corners) > 0:
                T_cam_marker = cam.estimate_pose(corners, idx=0)  # mm
                if T_cam_marker is not None:
                    current_T_cam = T_cam_marker  # camera->marker, mm

                    # IMPORTANT: get fresh base->tcp from robot (for accurate transform at this instant)
                    T_base_tcp_robot = ur.get_current_base_tcp()
                    if T_base_tcp_robot is not None:
                        # assume robot returns meters; convert to mm for chain multiplication
                        T_base_tcp_m = T_base_tcp_robot.copy()
                        T_base_tcp_mm = T_base_tcp_m.copy()
                        T_base_tcp_mm[:3, 3] = T_base_tcp_m[:3, 3] * 1000.0
                    else:
                        # fallback to last known T_base_tcp_mm (set earlier)
                        logging.debug("ur.get_current_base_tcp() returned None; using last known T_base_tcp_mm.")

                    # Convert camera->marker to base->marker (still mm)
                    current_T_base_mm = cam_marker_to_base_marker(current_T_cam, T_base_tcp_mm, T_tcp_cam)
                    detected_and_valid = True
                    last_detection = time.time()

                    # Compute PBVS velocity in base frame (controller expects mm input)
                    v_lin_base, v_ang_base = controller.compute_velocity(current_T_base_mm, desired_T_base_mm, dt=rate)
                    # NOTE: compute_velocity returns v_lin in m/s, v_ang in rad/s

                    # Now get robot Jacobian (expected to map qdot -> [v_lin_base; v_ang_base] in base-frame)
                    J = ur.get_jacobian()  # expected shape (6, n_joints), units matching (m/s, rad/s)
                    if J is None:
                        # No Jacobian available: fallback to sending Cartesian velocity directly if supported
                        logging.warning("Robot Jacobian unavailable; attempting to send Cartesian twist directly (if supported).")
                        # Option A: if UR adapter supports sending Cartesian twist in base frame or tool frame:
                        # Here we attempt to send in tool-frame by converting base->tool (if T_base_tcp_m available)
                        if hasattr(ur, "send_cartesian_velocity"):
                            # convert base-frame linear/ang velocities to tool-frame for sending
                            # note T_base_tcp_m is in meters; v_lin_base in m/s; v_ang_base in rad/s
                            if 'T_base_tcp_m' in locals():
                                v_lin_tool, v_ang_tool = controller.base_to_tool_velocity(v_lin_base, v_ang_base, T_base_tcp_m)
                                ur.send_cartesian_velocity(v_lin_tool, v_ang_tool, rate)
                            else:
                                # cannot convert (no T_base_tcp_m), send base-frame velocities directly (may be unsupported)
                                ur.send_cartesian_velocity(v_lin_base, v_ang_base, rate)
                        else:
                            logging.error("UR adapter has no method to send Cartesian velocity. Skipping command.")
                    else:
                        # We have Jacobian J (6 x n). Compute joint velocities using Damped Least Squares.
                        try:
                            q_dot = PBVSController.map_cartesian_to_joint_velocities(v_lin_base, v_ang_base, J, damping=1e-2)
                        except Exception as ex:
                            logging.exception("Failed to map cartesian to joint velocities: %s", ex)
                            q_dot = None

                        if q_dot is not None:
                            # send joint velocity command to robot
                            if hasattr(ur, "send_joint_velocity"):
                                ur.send_joint_velocity(q_dot)
                            else:
                                # if UR adapter lacks send_joint_velocity, as fallback try Cartesian send in tool-frame
                                logging.warning("URAdapter missing send_joint_velocity; falling back to Cartesian send if available.")
                                if 'T_base_tcp_m' in locals() and hasattr(ur, "send_cartesian_velocity"):
                                    v_lin_tool, v_ang_tool = controller.base_to_tool_velocity(v_lin_base, v_ang_base, T_base_tcp_m)
                                    ur.send_cartesian_velocity(v_lin_tool, v_ang_tool, rate)
                                else:
                                    logging.error("No suitable send method found on URAdapter; command dropped.")
                else:
                    logging.debug("solvePnP returned no valid pose for detected marker.")
            # safety stop if no detection for timeout
            if not detected_and_valid:
                if time.time() - last_detection > cfg.safety_timeout_s:
                    # If robot supports explicit zeroing of joint velocities, use it; else send zero cartesian twist
                    if hasattr(ur, "send_joint_velocity"):
                        n_j = ur.get_num_joints() if hasattr(ur, "get_num_joints") else None
                        if n_j:
                            ur.send_joint_velocity(np.zeros(n_j))
                        else:
                            # fallback to sending zero cartesian twist
                            if hasattr(ur, "send_cartesian_velocity"):
                                ur.send_cartesian_velocity(np.zeros(3), np.zeros(3), rate)
                    else:
                        if hasattr(ur, "send_cartesian_velocity"):
                            ur.send_cartesian_velocity(np.zeros(3), np.zeros(3), rate)

            # GUI and key handling
            if cfg.show_gui:
                cam.draw_marker_overlay(frame, corners, ids)
                # show base-frame pose if available
                if current_T_base_mm is not None:
                    R = current_T_base_mm[:3, :3]
                    t_mm = current_T_base_mm[:3, 3]
                    roll, pitch, yaw = rotation_matrix_to_euler(R)
                    text = f"base t(mm): {t_mm.round(1)} rpy(deg): {[round(np.degrees(x),2) for x in (roll,pitch,yaw)]}"
                    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.imshow("UR P
