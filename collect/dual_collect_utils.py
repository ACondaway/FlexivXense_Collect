import json
import os
import sys
import time
from argparse import Namespace
from datetime import datetime
from typing import Any, Dict, Mapping, Optional

import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from r3kit.devices.camera.realsense import config as rs_cfg
from r3kit.devices.camera.realsense.d415 import D415


FPS = 30
D415_CAMERAS = {
    "cam_327322062498": "327322062498",
}


def create_session_dirs(
    save_root: str,
    d415_cameras: Optional[Mapping[str, str]] = None,
    session_name: Optional[str] = None,
    xense_sensors: Optional[Mapping[str, object]] = None,
) -> str:
    os.makedirs(save_root, exist_ok=True)

    if session_name is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_name = f"record_{timestamp}"

    session_dir = os.path.join(save_root, session_name)
    os.makedirs(session_dir, exist_ok=True)

    # camera directories
    if d415_cameras:
        for cam_name in d415_cameras.keys():
            cam_dir = os.path.join(session_dir, cam_name)
            os.makedirs(cam_dir, exist_ok=True)
            os.makedirs(os.path.join(cam_dir, "color"), exist_ok=True)
            os.makedirs(os.path.join(cam_dir, "depth"), exist_ok=True)

    # data collection directories
    os.makedirs(os.path.join(session_dir, "tcps"), exist_ok=True)
    os.makedirs(os.path.join(session_dir, "angles"), exist_ok=True)
    os.makedirs(os.path.join(session_dir, "master_tcps"), exist_ok=True)
    os.makedirs(os.path.join(session_dir, "master_angles"), exist_ok=True)
    os.makedirs(os.path.join(session_dir, "timestamps"), exist_ok=True)

    # xense tactile sensor directories
    if xense_sensors:
        for xense_name in xense_sensors.keys():
            for subdir in ("rectify", "depth"):
                os.makedirs(os.path.join(session_dir, xense_name, subdir), exist_ok=True)

    print(f"Data will be saved to: {session_dir}")
    return session_dir


def write_metadata(session_dir: str, metadata: Any) -> str:
    if isinstance(metadata, Namespace):
        metadata = vars(metadata)
    else:
        metadata = dict(metadata)

    metadata.setdefault("created_at", datetime.now().isoformat(timespec="seconds"))
    metadata_path = os.path.join(session_dir, "metadata.json")
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False, default=str)
    return metadata_path


def init_cameras(
    d415_cameras: Optional[Mapping[str, str]] = None,
    fps: int = FPS,
) -> Dict[str, D415]:
    if d415_cameras is None:
        d415_cameras = D415_CAMERAS

    rs_cfg.D415_STREAMS = [
        (rs.stream.depth, 640, 480, rs.format.z16, fps),
        (rs.stream.color, 640, 480, rs.format.bgr8, fps),
    ]

    return {
        cam_name: D415(id=serial, depth=True, name=cam_name)
        for cam_name, serial in d415_cameras.items()
    }


def init_xense(gripper_id: str, name: str = "Xense"):
    from r3kit.devices.gripper.xense.xense import Xense

    gripper = Xense(id=gripper_id, name=name)
    gripper.block(blocking=False)
    return gripper


def init_xense_camera(device_id: str, name: str = "xense", fps: int = 50, mac_addr: str = None):
    """Initialise a Xense tactile camera sensor (end-effector, not gripper)."""
    from r3kit.devices.camera.xense.xense import Xense as XenseCam

    return XenseCam(device_id=device_id, mac_addr=mac_addr, name=name, fps=fps)


class AnglerGripperController:
    def __init__(
        self,
        encoder,
        open_angle: float,
        close_angle: float,
        open_width: float,
        close_width: float,
    ) -> None:
        if open_angle == close_angle:
            raise ValueError("open_angle and close_angle must be different")
        self.encoder = encoder
        self.open_angle = float(open_angle)
        self.close_angle = float(close_angle)
        self.open_width = float(open_width)
        self.close_width = float(close_width)

    def read(self) -> float:
        angle = float(np.asarray(self.encoder.get()["angle"]).reshape(-1)[0])
        ratio = (angle - self.close_angle) / (self.open_angle - self.close_angle)
        ratio = float(np.clip(ratio, 0.0, 1.0))
        return self.close_width + ratio * (self.open_width - self.close_width)


def init_angler_controller(
    encoder_id: str,
    index: int,
    baudrate: int,
    gap: float,
    strict: bool,
    open_angle: float,
    close_angle: float,
    open_width: float,
    close_width: float,
    name: str = "master_angler",
) -> AnglerGripperController:
    from r3kit.devices.encoder.pdcd.angler import Angler

    encoder = Angler(
        id=encoder_id,
        index=[index],
        baudrate=baudrate,
        gap=gap,
        strict=strict,
        name=name,
    )
    return AnglerGripperController(
        encoder=encoder,
        open_angle=open_angle,
        close_angle=close_angle,
        open_width=open_width,
        close_width=close_width,
    )


def save_camera_frames(
    cameras: Mapping[str, D415],
    session_dir: str,
    frame_idx: int,
) -> None:
    for name, cam in cameras.items():
        color_frame, depth_frame = cam.get()

        if color_frame is not None and depth_frame is not None:
            cv2.imwrite(
                os.path.join(session_dir, name, "color", f"{frame_idx:016d}.png"),
                color_frame,
            )
            cv2.imwrite(
                os.path.join(session_dir, name, "depth", f"{frame_idx:016d}.png"),
                depth_frame,
            )


def collect_teleop_data(
    state_reader,
    slave_gripper,
    cameras: Mapping[str, D415],
    session_dir: str,
    stop_event,
    fps: int = FPS,
    use_gripper: bool = True,
    status_period: int = 100,
) -> None:
    rate_control = RateControl(fps)
    frame_idx = 0

    print("Start data collection...")

    while not stop_event.is_set():
        actual_rate = rate_control.sleep()

        # camera data collection
        save_camera_frames(cameras, session_dir, frame_idx)

        # TCP pose, joint angle, and gripper data collection
        tcp_xyz, tcp_quat_xyzw, slave_joint_angles = state_reader.read_saved_xyzquat()
        slave_gripper_width = slave_gripper.read() if use_gripper else 0.0

        pose_data = np.concatenate([tcp_xyz, tcp_quat_xyzw, [slave_gripper_width]])
        joint_data = np.concatenate([slave_joint_angles, [slave_gripper_width]])

        pose_path = os.path.join(session_dir, "tcps", f"tcp_{frame_idx:05d}.npy")
        joint_path = os.path.join(session_dir, "angles", f"angle_{frame_idx:05d}.npy")
        np.save(pose_path, pose_data)
        np.save(joint_path, joint_data)

        if status_period and frame_idx % status_period == 0:
            print(f"Actual rate: {actual_rate:.2f} Hz, collected frames: {frame_idx}")

        frame_idx += 1


def collect_teleop_data_parallel(
    state_reader,
    slave_gripper,
    cameras,
    session_dir: str,
    stop_event,
    fps: int = FPS,
    use_gripper: bool = True,
    num_writers: int = 4,
    png_level: int = 1,
    status_period: int = 100,
    xense_sensors=None,   # Optional[Dict[str, XenseCam]]
    xense_fps: int = 50,
) -> None:
    """
    True async data collection: camera, robot, and xense each run independently.

    CameraCapture thread  — paced by hardware frame delivery (GIL released)
    RobotCapture thread   — rate-controlled at fps Hz
    XenseCapture thread   — rate-controlled at xense_fps Hz (one per sensor)
    ParallelWriter pool   — N subprocesses handle all disk I/O

    Each thread records its own perf_counter timestamps independently.
    Postprocessing aligns streams by nearest-neighbour timestamp matching.
    """
    from parallel_collect_utils import CameraCapture, RobotCapture, XenseCapture, ParallelWriter

    writer = ParallelWriter(num_workers=num_writers)
    writer.start()

    cam_captures = {
        name: CameraCapture(cam, name, session_dir, writer, png_level)
        for name, cam in cameras.items()
    }
    robot_capture = RobotCapture(
        state_reader, slave_gripper, session_dir, writer, fps, use_gripper
    )
    xense_captures = {}
    if xense_sensors:
        xense_captures = {
            name: XenseCapture(xense, name, session_dir, writer, xense_fps)
            for name, xense in xense_sensors.items()
        }

    for cc in cam_captures.values():
        cc.start()
    robot_capture.start()
    for xc in xense_captures.values():
        xc.start()

    all_streams = " | ".join(
        [f"cam({list(cam_captures.keys())})", f"robot({fps}Hz)"]
        + [f"xense_{n}({xense_fps}Hz)" for n in xense_captures]
    )
    print(f"[ParallelCollect] {all_streams} | writers: {num_writers}")

    status_interval = status_period / fps if fps > 0 else 3.0

    try:
        while True:
            signaled = stop_event.wait(timeout=status_interval)
            if signaled:
                break
            parts = [f"{n}:{cc.frame_count}" for n, cc in cam_captures.items()]
            parts += [f"robot:{robot_capture.frame_count}"]
            parts += [f"{n}:{xc.frame_count}" for n, xc in xense_captures.items()]
            print(f"[ParallelCollect] {' | '.join(parts)} | queue:{writer.queue_size()}")
    finally:
        for cc in cam_captures.values():
            cc.stop()
        robot_capture.stop()
        for xc in xense_captures.values():
            xc.stop()

        ts_dir = os.path.join(session_dir, "timestamps")
        for cc in cam_captures.values():
            cc.save_timestamps(ts_dir)
        robot_capture.save_timestamps(ts_dir)
        for xc in xense_captures.values():
            xc.save_timestamps(ts_dir)

        pending = writer.queue_size()
        print(f"[ParallelCollect] Flushing {pending} pending writes...")
        writer.flush_and_stop()

        parts = [f"{n}:{cc.frame_count}" for n, cc in cam_captures.items()]
        parts += [f"robot:{robot_capture.frame_count}"]
        parts += [f"{n}:{xc.frame_count}" for n, xc in xense_captures.items()]
        print(f"[ParallelCollect] Done. {' | '.join(parts)}")


class RateControl:
    def __init__(self, rate_hz):
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz
        self.last_time = time.time()
        self.actual_rate = 0
        self.frame_count = 0
        self.start_time = time.time()

    def sleep(self):
        now = time.time()
        elapsed = now - self.last_time
        sleep_time = self.interval - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)

        self.last_time = time.time()
        self.frame_count += 1

        total_time = time.time() - self.start_time
        if total_time > 0:
            self.actual_rate = self.frame_count / total_time

        return self.actual_rate
