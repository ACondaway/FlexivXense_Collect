"""
parallel_collect_utils.py

True async, multi-threaded data collection pipeline.

Architecture
------------

  CameraCapture thread (1 per camera)
      cam.get()  ← blocks at hardware frame rate (~30 fps, releases GIL)
      t_cam = perf_counter()
      writer.submit(WriteBundle(images=[...]))
                │
  RobotCapture thread (1 total)              ← completely independent
      rate_control.sleep()
      read_both_saved_xyzquat()  ← TDK cached state, <1 ms
      t_robot = perf_counter()
      writer.submit(WriteBundle(npys=[...]))
                │
                ▼
          mp.Queue  (up to 400 items)
     ↙      ↓       ↘       ↘
 Writer-0 Writer-1 Writer-2 Writer-3    ← subprocesses (PNG compression)

Camera and robot run at their own natural rates and are never blocked by
each other.  Timestamps are saved independently; postprocessing aligns
them by nearest-neighbour matching (see postprocess/verify_utils.py).
"""

from __future__ import annotations

import multiprocessing as mp
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Data containers (must be picklable for mp.Queue)
# ---------------------------------------------------------------------------

@dataclass
class ImageWrite:
    path: str
    img: np.ndarray
    png_level: int = 1   # 0 = no compression, 9 = max


@dataclass
class NpyWrite:
    path: str
    arr: np.ndarray


@dataclass
class WriteBundle:
    """All disk writes for one captured item (camera frame or robot state)."""
    images: List[ImageWrite] = field(default_factory=list)
    npys:   List[NpyWrite]   = field(default_factory=list)


# ---------------------------------------------------------------------------
# Disk-writer subprocess
# ---------------------------------------------------------------------------

def _writer_worker(queue: mp.Queue, stop_event: mp.Event) -> None:
    """
    Drain the write queue until stop_event is set AND the queue is empty.
    Never drops items — all in-flight bundles are written before exit.
    """
    while True:
        try:
            bundle: WriteBundle = queue.get(timeout=0.05)
        except Exception:
            if stop_event.is_set():
                break
            continue

        for iw in bundle.images:
            cv2.imwrite(iw.path, iw.img, [cv2.IMWRITE_PNG_COMPRESSION, iw.png_level])
        for nw in bundle.npys:
            np.save(nw.path, nw.arr)


# ---------------------------------------------------------------------------
# Writer pool
# ---------------------------------------------------------------------------

class ParallelWriter:
    """
    Pool of disk-writer subprocesses fed by a shared mp.Queue.

    Both CameraCapture and RobotCapture threads submit to the same pool.
    Uses spawn context so camera/robot SDK handles are not inherited.
    """

    def __init__(self, num_workers: int = 4, queue_maxsize: int = 400) -> None:
        self.num_workers = num_workers
        self._ctx = mp.get_context("spawn")
        self._queue: mp.Queue = self._ctx.Queue(maxsize=queue_maxsize)
        self._stop_event: mp.Event = self._ctx.Event()
        self._workers: list = []

    def start(self) -> None:
        for _ in range(self.num_workers):
            p = self._ctx.Process(
                target=_writer_worker,
                args=(self._queue, self._stop_event),
                daemon=True,
            )
            p.start()
            self._workers.append(p)

    def submit(self, bundle: WriteBundle, timeout: float = 5.0) -> None:
        """Non-blocking enqueue (blocks only if the queue is full)."""
        self._queue.put(bundle, block=True, timeout=timeout)

    def queue_size(self) -> int:
        return self._queue.qsize()

    def flush_and_stop(self, timeout: float = 60.0) -> None:
        """Signal workers to finish draining, then join."""
        self._stop_event.set()
        for p in self._workers:
            p.join(timeout=timeout)
        self._workers.clear()


# ---------------------------------------------------------------------------
# Camera capture thread
# ---------------------------------------------------------------------------

class CameraCapture:
    """
    Dedicated thread for one RealSense camera.

    cam.get() calls pipeline.wait_for_frames() which releases the GIL and
    blocks until the next hardware frame arrives (~33 ms at 30 fps).  This
    thread is therefore naturally paced by the camera hardware — no sleep
    needed — and never blocks the robot capture thread.

    Timestamps (perf_counter seconds, recorded immediately after each
    cam.get() returns) are accumulated in memory and flushed to disk when
    stop() is called.
    """

    def __init__(
        self,
        cam,
        cam_name: str,
        session_dir: str,
        writer: ParallelWriter,
        png_level: int = 1,
    ) -> None:
        self.cam = cam
        self.cam_name = cam_name
        self.session_dir = session_dir
        self.writer = writer
        self.png_level = png_level

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"cam-{cam_name}", daemon=True)
        self._timestamps: List[float] = []
        self._frame_idx: int = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        """Signal the thread to stop and wait for it to finish its current frame."""
        self._stop.set()
        self._thread.join()

    @property
    def frame_count(self) -> int:
        return self._frame_idx

    def save_timestamps(self, ts_dir: str) -> None:
        """Flush accumulated timestamps to disk (call after stop())."""
        path = os.path.join(ts_dir, f"cam_{self.cam_name}_timestamps.npy")
        np.save(path, np.array(self._timestamps, dtype=np.float64))

    def _run(self) -> None:
        while not self._stop.is_set():
            color_frame, depth_frame = self.cam.get()
            t = time.perf_counter()
            self._timestamps.append(t)

            images = []
            if color_frame is not None:
                images.append(ImageWrite(
                    path=os.path.join(
                        self.session_dir, self.cam_name, "color",
                        f"{self._frame_idx:016d}.png",
                    ),
                    img=color_frame,
                    png_level=self.png_level,
                ))
            if depth_frame is not None:
                images.append(ImageWrite(
                    path=os.path.join(
                        self.session_dir, self.cam_name, "depth",
                        f"{self._frame_idx:016d}.png",
                    ),
                    img=depth_frame,
                    png_level=self.png_level,
                ))

            if images:
                self.writer.submit(WriteBundle(images=images))

            self._frame_idx += 1


# ---------------------------------------------------------------------------
# Robot state capture thread
# ---------------------------------------------------------------------------

class RobotCapture:
    """
    Dedicated thread for master + slave arm state and gripper.

    Reads robot state at ``fps`` Hz using a tight rate-control loop.
    read_both_saved_xyzquat() reads TDK's cached state (<1 ms, releases GIL)
    and is completely independent of the camera threads.

    Timestamps (perf_counter seconds, recorded immediately after each state
    read) are accumulated in memory and flushed to disk when stop() is called.
    """

    def __init__(
        self,
        state_reader,
        slave_gripper,
        session_dir: str,
        writer: ParallelWriter,
        fps: int = 30,
        use_gripper: bool = True,
    ) -> None:
        self.state_reader = state_reader
        self.slave_gripper = slave_gripper
        self.session_dir = session_dir
        self.writer = writer
        self.fps = fps
        self.use_gripper = use_gripper

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="robot-capture", daemon=True)
        self._timestamps: List[float] = []
        self._frame_idx: int = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()

    @property
    def frame_count(self) -> int:
        return self._frame_idx

    def save_timestamps(self, ts_dir: str) -> None:
        """Flush accumulated timestamps to disk (call after stop())."""
        path = os.path.join(ts_dir, "robot_timestamps.npy")
        np.save(path, np.array(self._timestamps, dtype=np.float64))

    def _run(self) -> None:
        interval = 1.0 / self.fps
        last_t = time.perf_counter()

        while not self._stop.is_set():
            # Rate control — inline to avoid circular import with dual_collect_utils
            now = time.perf_counter()
            wait = interval - (now - last_t)
            if wait > 0:
                time.sleep(wait)
            last_t = time.perf_counter()

            (
                master_xyz, master_quat, master_joints,
                slave_xyz,  slave_quat,  slave_joints,
            ) = self.state_reader.read_both_saved_xyzquat()
            slave_gripper_width = self.slave_gripper.read() if self.use_gripper else 0.0

            t = time.perf_counter()
            self._timestamps.append(t)

            idx = self._frame_idx
            slave_pose   = np.concatenate([slave_xyz,  slave_quat,  [slave_gripper_width]])
            slave_joint  = np.concatenate([slave_joints,             [slave_gripper_width]])
            master_pose  = np.concatenate([master_xyz, master_quat, [slave_gripper_width]])
            master_joint = np.concatenate([master_joints,            [slave_gripper_width]])

            self.writer.submit(WriteBundle(npys=[
                NpyWrite(os.path.join(self.session_dir, "tcps",
                                      f"tcp_{idx:05d}.npy"),   slave_pose.copy()),
                NpyWrite(os.path.join(self.session_dir, "angles",
                                      f"angle_{idx:05d}.npy"), slave_joint.copy()),
                NpyWrite(os.path.join(self.session_dir, "master_tcps",
                                      f"tcp_{idx:05d}.npy"),   master_pose.copy()),
                NpyWrite(os.path.join(self.session_dir, "master_angles",
                                      f"angle_{idx:05d}.npy"), master_joint.copy()),
            ]))

            self._frame_idx += 1


# ---------------------------------------------------------------------------
# Xense tactile sensor capture thread
# ---------------------------------------------------------------------------

class XenseCapture:
    """
    Dedicated capture thread for one Xense tactile sensor (end-effector).

    Unlike D415, Xense.get() is non-blocking (reads the latest sensor frame
    synchronously from the SDK buffer), so this thread uses explicit rate
    control — similar to RobotCapture.

    Rectify data is saved as .npy (numerical tactile array, not a viewable
    image) to preserve full precision and dtype.
    """

    def __init__(
        self,
        xense,
        xense_name: str,
        session_dir: str,
        writer: ParallelWriter,
        fps: int = 50,
    ) -> None:
        self.xense = xense
        self.xense_name = xense_name
        self.session_dir = session_dir
        self.writer = writer
        self.fps = fps

        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name=f"xense-{xense_name}", daemon=True
        )
        self._timestamps: List[float] = []
        self._frame_idx: int = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()

    @property
    def frame_count(self) -> int:
        return self._frame_idx

    def save_timestamps(self, ts_dir: str) -> None:
        path = os.path.join(ts_dir, f"xense_{self.xense_name}_timestamps.npy")
        np.save(path, np.array(self._timestamps, dtype=np.float64))

    def _run(self) -> None:
        from xensesdk import Sensor

        interval = 1.0 / self.fps
        last_t = time.perf_counter()
        base = os.path.join(self.session_dir, self.xense_name)

        while not self._stop.is_set():
            now = time.perf_counter()
            wait = interval - (now - last_t)
            if wait > 0:
                time.sleep(wait)
            last_t = time.perf_counter()

            rectify, depth = self.xense.sensor.selectSensorInfo(
                Sensor.OutputType.Rectify,
                Sensor.OutputType.Depth,
            )
            t = time.perf_counter()
            self._timestamps.append(t)

            idx = self._frame_idx
            npys = []
            for subdir, arr in (("rectify", rectify), ("depth", depth)):
                if arr is not None:
                    npys.append(NpyWrite(
                        path=os.path.join(base, subdir, f"{idx:05d}.npy"),
                        arr=arr.copy(),
                    ))
            if npys:
                self.writer.submit(WriteBundle(npys=npys))

            self._frame_idx += 1
