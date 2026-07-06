#!/usr/bin/env python3
"""
convert_to_hdf5.py — Parallel post-processing: raw session dirs → HDF5 episodes.

Raw schema (per session_dir/)
─────────────────────────────
  cam_{name}/color/{frame:016d}.png       → observation/{role}/rgb   (JPEG bytes)
  cam_{name}/depth/{frame:016d}.png       → (not in target schema, skipped)
  tcps/tcp_{idx:05d}.npy                  → actor/slot[:, :7] + embodiment/ee[:, :7]
                                             (shape (8,): xyz·3 + quat_xyzw·4 + gripper·1)
  angles/angle_{idx:05d}.npy             → (not written; joint omitted per schema)
  master_tcps/tcp_{idx:05d}.npy          → actor/prism[:, :7]
  master_angles/angle_{idx:05d}.npy      → (not in target schema, skipped)
  timestamps/cam_{name}_timestamps.npy   → used for nearest-neighbour alignment
  timestamps/robot_timestamps.npy        → reference timeline
  timestamps/xense_{name}_timestamps.npy → used for nearest-neighbour alignment
  xense_left/rectify/{idx:05d}.npy       → tactile/left_gsmini/rgb   (JPEG bytes)
  xense_left/depth/{idx:05d}.npy         → tactile/left_gsmini/depth (float32)
  xense_right/rectify/{idx:05d}.npy      → tactile/right_gsmini/rgb
  xense_right/depth/{idx:05d}.npy        → tactile/right_gsmini/depth

Target HDF5 schema (matches data/1.hdf5)
─────────────────────────────────────────
  actor/prism         (N, 7)  float32   master xyz+quat_xyzw
  actor/slot          (N, 7)  float32   slave  xyz+quat_xyzw
  atom/id             (N,)    int64     episode index (per-file constant)
  atom/tag            (N,)    |S5       b"move" for all teleoperation frames
  embodiment/ee       (N, 7)  float32   slave end-effector xyz+quat_xyzw
  embodiment/ee       (N, 7)  float32   slave end-effector xyz+quat_xyzw
  observation/{role}/rgb  (N,)  |S{max}  JPEG bytes (role: head)
  step                (N,)    int64     [0, 1, …, N-1]
  tactile/{role}/depth    (N,H,W)  float32
  tactile/{role}/rgb      (N,)    |S{max}  JPEG bytes from rectify

Parallelism
───────────
  Level 1 — session-level: multiprocessing.Pool (one process per session)
  Level 2 — image encoding: ThreadPoolExecutor inside each session worker

Usage
─────
  # Convert all sessions under /data/raw → /data/hdf5/
  conda run -n collection python3 postprocess/convert_to_hdf5.py \\
      --save-root /data/raw \\
      --out-dir   /data/hdf5 \\
      --workers   8

  # Single session
  conda run -n collection python3 postprocess/convert_to_hdf5.py \\
      --save-root /data/raw/record_20240101_120000_000000 \\
      --out-dir   /data/hdf5 \\
      --workers   1

  # Single head camera only
  conda run -n collection python3 postprocess/convert_to_hdf5.py \\
      --save-root /data/raw \\
      --out-dir   /data/hdf5 \\
      --cam-role  cam_327322062498=head
"""

from __future__ import annotations

import argparse
import logging
import multiprocessing as mp
import os
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from glob import glob
from typing import Dict, List, Optional, Tuple

import cv2
import h5py
import numpy as np

logger = logging.getLogger("convert_to_hdf5")

# ─── defaults ────────────────────────────────────────────────────────────────

JPEG_QUALITY = 95
N_ENCODE_THREADS = 4          # threads per session worker for JPEG encoding

# Default xense dir-name → HDF5 tactile role mapping
DEFAULT_XENSE_ROLE: Dict[str, str] = {
    "xense_left":  "left_gsmini",
    "xense_right": "right_gsmini",
}

# When multiple cameras found, assign roles in this order (sorted cam dir names)
# Current setup: single head camera only.
DEFAULT_CAM_ROLES = ["head"]


# ─── small utilities ─────────────────────────────────────────────────────────

def align_nearest(ref_ts: np.ndarray, target_ts: np.ndarray) -> np.ndarray:
    """
    For each timestamp in ref_ts find the index of the nearest value in
    target_ts.  O(N log M) via searchsorted.
    """
    idx = np.searchsorted(target_ts, ref_ts)
    il = np.clip(idx - 1, 0, len(target_ts) - 1)
    ir = np.clip(idx,     0, len(target_ts) - 1)
    dl = np.abs(ref_ts - target_ts[il])
    dr = np.abs(ref_ts - target_ts[ir])
    return np.where(dl <= dr, il, ir)


def encode_jpeg(img: np.ndarray, quality: int) -> bytes:
    """BGR (H,W,3) or gray (H,W) ndarray → JPEG bytes."""
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def encode_jpeg_batch(
    images: List[np.ndarray],
    quality: int = JPEG_QUALITY,
    n_threads: int = N_ENCODE_THREADS,
) -> List[bytes]:
    """Encode a list of images to JPEG bytes in parallel."""
    fn = partial(encode_jpeg, quality=quality)
    with ThreadPoolExecutor(max_workers=n_threads) as pool:
        return list(pool.map(fn, images))


def pack_bytes(encoded: List[bytes]) -> np.ndarray:
    """
    Pack variable-length byte strings into a fixed-width numpy S-array,
    matching the |S{max_size} dtype used in the reference HDF5.
    numpy automatically null-pads shorter entries.
    """
    max_len = max(len(b) for b in encoded)
    return np.array(encoded, dtype=f"S{max_len}")


def sorted_glob(directory: str, pattern: str) -> List[str]:
    return sorted(glob(os.path.join(directory, pattern)))


# ─── session discovery ───────────────────────────────────────────────────────

def is_session_dir(path: str) -> bool:
    return (
        os.path.isdir(path)
        and os.path.isdir(os.path.join(path, "tcps"))
        and os.path.isdir(os.path.join(path, "angles"))
    )


def find_sessions(root: str) -> List[str]:
    """
    Return sorted list of valid session directories.
    If root itself is a session, return [root].
    Otherwise scan one level deep.
    """
    if is_session_dir(root):
        return [root]
    return [
        os.path.join(root, name)
        for name in sorted(os.listdir(root))
        if is_session_dir(os.path.join(root, name))
    ]


def detect_cameras(session_dir: str) -> List[str]:
    """Sorted list of camera subdirectory names that have a color/ subfolder."""
    return sorted(
        d for d in os.listdir(session_dir)
        if d.startswith("cam_")
        and os.path.isdir(os.path.join(session_dir, d, "color"))
    )


def detect_xense(session_dir: str) -> List[str]:
    """Sorted list of xense subdirectory names."""
    return sorted(
        d for d in os.listdir(session_dir)
        if d.startswith("xense_")
        and os.path.isdir(os.path.join(session_dir, d))
    )


# ─── data loading ────────────────────────────────────────────────────────────

def load_robot_data(
    session_dir: str,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Returns
    -------
    slave_pose  : (N, 8) float32 — xyz·3 + quat_xyzw·4 + gripper·1
    master_pose : (N, 8) float32 or None
    """
    tcp_paths  = sorted_glob(os.path.join(session_dir, "tcps"), "tcp_*.npy")
    slave_pose = np.stack([np.load(p) for p in tcp_paths], axis=0).astype(np.float32)

    master_pose = None
    mtcp_dir = os.path.join(session_dir, "master_tcps")
    if os.path.isdir(mtcp_dir):
        mpaths = sorted_glob(mtcp_dir, "tcp_*.npy")
        if mpaths:
            master_pose = np.stack([np.load(p) for p in mpaths], axis=0).astype(np.float32)

    return slave_pose, master_pose


def load_all_timestamps(session_dir: str) -> Dict[str, np.ndarray]:
    """Load all *.npy files from timestamps/ keyed by filename stem."""
    ts_dir = os.path.join(session_dir, "timestamps")
    result: Dict[str, np.ndarray] = {}
    if not os.path.isdir(ts_dir):
        return result
    for p in glob(os.path.join(ts_dir, "*.npy")):
        stem = os.path.splitext(os.path.basename(p))[0]
        result[stem] = np.load(p)
    return result


def build_frame_alignment(
    robot_ts: Optional[np.ndarray],
    all_ts: Dict[str, np.ndarray],
    n_robot: int,
    cam_dirs: List[str],
    xense_dirs: List[str],
    session_dir: str,
) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    """
    For each robot frame index, find the nearest camera/xense frame index.

    Timestamp file naming convention (from parallel_collect_utils.py):
      cameras : cam_{cam_name}_timestamps.npy
      xense   : xense_{xense_name}_timestamps.npy
      robot   : robot_timestamps.npy

    Falls back to sequential indexing (clipped) when timestamps are missing.
    """
    robot_idx = np.arange(n_robot)

    cam_align: Dict[str, np.ndarray] = {}
    for cam in cam_dirs:
        n_cam = len(sorted_glob(os.path.join(session_dir, cam, "color"), "*.png"))
        ts_key = f"cam_{cam}_timestamps"
        cam_ts = all_ts.get(ts_key)
        if robot_ts is not None and cam_ts is not None and len(cam_ts) > 0:
            cam_align[cam] = np.clip(align_nearest(robot_ts, cam_ts), 0, n_cam - 1)
        else:
            cam_align[cam] = np.clip(robot_idx, 0, n_cam - 1)

    xense_align: Dict[str, np.ndarray] = {}
    for xd in xense_dirs:
        depth_dir = os.path.join(session_dir, xd, "depth")
        n_xense = len(sorted_glob(depth_dir, "*.npy")) if os.path.isdir(depth_dir) else 0
        if n_xense == 0:
            continue
        ts_key = f"xense_{xd}_timestamps"
        xts = all_ts.get(ts_key)
        if robot_ts is not None and xts is not None and len(xts) > 0:
            xense_align[xd] = np.clip(align_nearest(robot_ts, xts), 0, n_xense - 1)
        else:
            xense_align[xd] = np.clip(robot_idx, 0, n_xense - 1)

    return cam_align, xense_align


# ─── image loading ───────────────────────────────────────────────────────────

def load_color_frames(
    session_dir: str,
    cam_name: str,
    indices: np.ndarray,
) -> List[np.ndarray]:
    paths = sorted_glob(os.path.join(session_dir, cam_name, "color"), "*.png")
    imgs = []
    for i in indices:
        img = cv2.imread(paths[i])
        if img is None:
            raise RuntimeError(f"Failed to read {paths[i]}")
        imgs.append(img)
    return imgs


def load_rectify_frames(
    session_dir: str,
    xense_name: str,
    indices: np.ndarray,
) -> List[np.ndarray]:
    """
    Load rectify .npy arrays and normalise to uint8 for JPEG encoding.
    Handles float [0,1], float (0,255], and uint8 inputs.
    """
    paths = sorted_glob(os.path.join(session_dir, xense_name, "rectify"), "*.npy")
    imgs = []
    for i in indices:
        arr = np.load(paths[i])
        if arr.dtype != np.uint8:
            if arr.max() <= 1.0 and arr.min() >= 0.0:
                arr = (arr * 255).astype(np.uint8)
            else:
                arr = np.clip(arr, 0, 255).astype(np.uint8)
        imgs.append(arr)
    return imgs


def load_depth_stack(
    session_dir: str,
    xense_name: str,
    indices: np.ndarray,
) -> np.ndarray:
    """Load aligned depth frames → (N, H, W) float32."""
    paths = sorted_glob(os.path.join(session_dir, xense_name, "depth"), "*.npy")
    return np.stack([np.load(paths[i]).astype(np.float32) for i in indices], axis=0)


# ─── core converter ──────────────────────────────────────────────────────────

def _convert_session(
    session_dir: str,
    out_path: str,
    episode_id: int,
    jpeg_quality: int,
    n_encode_threads: int,
    cam_role_map: Dict[str, str],
    xense_role_map: Dict[str, str],
    tag: bytes,
) -> None:
    # ── robot data ──────────────────────────────────────────────────────────
    slave_pose, master_pose = load_robot_data(session_dir)
    N = len(slave_pose)
    assert N > 0, "No robot frames found"
    assert N > 0, "No robot frames found"

    # ── timestamps & alignment ───────────────────────────────────────────────
    all_ts    = load_all_timestamps(session_dir)
    robot_ts  = all_ts.get("robot_timestamps")
    cam_dirs  = detect_cameras(session_dir)
    xense_dirs = detect_xense(session_dir)

    # Apply role maps — filter to known cams/xense only
    # cam_role_map: {cam_dir: role}   e.g. {"cam_xxx": "head"}
    # xense_role_map: {xense_dir: role}  e.g. {"xense_left": "left_gsmini"}
    active_cams   = [c for c in cam_dirs   if c in cam_role_map]
    active_xense  = [x for x in xense_dirs if x in xense_role_map]

    cam_align, xense_align = build_frame_alignment(
        robot_ts, all_ts, N, cam_dirs, active_xense, session_dir
    )

    # ── parallel image encoding ───────────────────────────────────────────────
    cam_encoded: Dict[str, List[bytes]] = {}
    for cam in active_cams:
        imgs = load_color_frames(session_dir, cam, cam_align[cam])
        cam_encoded[cam] = encode_jpeg_batch(imgs, jpeg_quality, n_encode_threads)

    xense_depth: Dict[str, np.ndarray] = {}
    xense_rgb: Dict[str, List[bytes]] = {}
    for xd in active_xense:
        idxs = xense_align[xd]
        xense_depth[xd] = load_depth_stack(session_dir, xd, idxs)
        rectify_dir = os.path.join(session_dir, xd, "rectify")
        if os.path.isdir(rectify_dir) and sorted_glob(rectify_dir, "*.npy"):
            imgs = load_rectify_frames(session_dir, xd, idxs)
            xense_rgb[xd] = encode_jpeg_batch(imgs, jpeg_quality, n_encode_threads)

    # ── write HDF5 ───────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with h5py.File(out_path, "w") as f:

        # actor
        prism_data = (
            master_pose[:, :7]
            if master_pose is not None
            else np.zeros((N, 7), dtype=np.float32)
        )
        f.create_dataset("actor/prism", data=prism_data,            dtype=np.float32)
        f.create_dataset("actor/slot",  data=slave_pose[:, :7],     dtype=np.float32)

        # atom
        f.create_dataset("atom/id",  data=np.full(N, episode_id, dtype=np.int64))
        f.create_dataset("atom/tag", data=np.array([tag] * N, dtype="S5"))

        # embodiment
        # embodiment — ee pose only (joint removed)
        f.create_dataset("embodiment/ee", data=slave_pose[:, :7], dtype=np.float32)

        # step
        f.create_dataset("step", data=np.arange(N, dtype=np.int64))

        # observation cameras
        for cam, role in cam_role_map.items():
            if cam in cam_encoded:
                f.create_dataset(
                    f"observation/{role}/rgb",
                    data=pack_bytes(cam_encoded[cam]),
                )

        # tactile
        for xd, role in xense_role_map.items():
            if xd not in active_xense:
                continue
            if xd in xense_depth:
                f.create_dataset(
                    f"tactile/{role}/depth",
                    data=xense_depth[xd],
                    dtype=np.float32,
                    chunks=(1, *xense_depth[xd].shape[1:]),  # one frame per chunk
                    compression="lzf",
                )
            if xd in xense_rgb:
                f.create_dataset(
                    f"tactile/{role}/rgb",
                    data=pack_bytes(xense_rgb[xd]),
                )

    logger.info("Wrote %d frames → %s", N, out_path)


def process_session_worker(args: Tuple) -> str:
    """
    Top-level function for each multiprocessing worker.
    Returns a status string ("OK: …" or "ERROR: …").
    """
    (
        session_dir, out_dir, episode_id,
        jpeg_quality, n_encode_threads,
        cam_role_map, xense_role_map, tag,
    ) = args

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    session_name = os.path.basename(session_dir)
    out_path = os.path.join(out_dir, f"{session_name}.hdf5")

    if os.path.exists(out_path):
        return f"SKIP (exists): {out_path}"

    try:
        _convert_session(
            session_dir, out_path, episode_id,
            jpeg_quality, n_encode_threads,
            cam_role_map, xense_role_map, tag,
        )
        return f"OK: {out_path}"
    except Exception as exc:
        import traceback
        return f"ERROR: {session_name}: {exc}\n{traceback.format_exc()}"


# ─── CLI ─────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Convert raw teleoperation session dirs to HDF5 (parallel)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--save-root", required=True,
        help="Root dir containing session subdirs, or a single session dir.",
    )
    p.add_argument(
        "--out-dir", required=True,
        help="Output directory for HDF5 files.",
    )
    p.add_argument(
        "--workers", type=int, default=min(8, mp.cpu_count()),
        help="Number of parallel processes (default: min(8, cpu_count)).",
    )
    p.add_argument(
        "--jpeg-quality", type=int, default=JPEG_QUALITY,
        help="JPEG quality 1–100 (default 95).",
    )
    p.add_argument(
        "--encode-threads", type=int, default=N_ENCODE_THREADS,
        help="Threads for JPEG encoding per worker (default 4).",
    )
    p.add_argument(
        "--cam-role", action="append", default=None, metavar="CAM_DIR=ROLE",
        help=(
            "Map a camera dir name to an observation role. "
            "E.g. --cam-role cam_327322062498=head. "
            "Can be repeated. Default: sorted cam dirs → head."
        ),
    )
    p.add_argument(
        "--xense-role", action="append", default=None, metavar="XENSE_DIR=ROLE",
        help=(
            "Map a xense dir to a tactile role. "
            "E.g. --xense-role xense_left=left_gsmini. "
            "Can be repeated. Default: xense_left→left_gsmini, xense_right→right_gsmini."
        ),
    )
    p.add_argument(
        "--tag", default="move",
        help="atom/tag string for all frames (max 5 chars, default 'move').",
    )
    p.add_argument(
        "--episode-start", type=int, default=0,
        help="Episode ID assigned to the first session (increments per session).",
    )
    p.add_argument(
        "--overwrite", action="store_true",
        help="Overwrite existing HDF5 files (default: skip).",
    )
    return p.parse_args()


def parse_role_map(entries: Optional[List[str]]) -> Dict[str, str]:
    if not entries:
        return {}
    result = {}
    for entry in entries:
        if "=" not in entry:
            raise ValueError(f"Expected KEY=VALUE, got: {entry!r}")
        k, v = entry.split("=", 1)
        result[k.strip()] = v.strip()
    return result


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    args = parse_args()
    tag_bytes = args.tag.encode()[:5]

    # ── find sessions ────────────────────────────────────────────────────────
    sessions = find_sessions(args.save_root)
    if not sessions:
        logger.error("No valid session dirs found under %s", args.save_root)
        return
    logger.info("Found %d session(s)", len(sessions))

    os.makedirs(args.out_dir, exist_ok=True)

    # ── build role maps ──────────────────────────────────────────────────────
    user_cam_roles   = parse_role_map(args.cam_role)
    user_xense_roles = parse_role_map(args.xense_role)

    # For each session, compute the effective cam_role_map:
    # - if user provided explicit mappings, use those
    # - otherwise auto-detect cameras in the first session and assign roles in order
    def effective_cam_roles(session_dir: str) -> Dict[str, str]:
        if user_cam_roles:
            return user_cam_roles
        cam_dirs = detect_cameras(session_dir)
        return {
            cam: DEFAULT_CAM_ROLES[i]
            for i, cam in enumerate(cam_dirs)
            if i < len(DEFAULT_CAM_ROLES)
        }

    def effective_xense_roles(session_dir: str) -> Dict[str, str]:
        if user_xense_roles:
            return user_xense_roles
        return DEFAULT_XENSE_ROLE.copy()

    # ── build worker arg list ────────────────────────────────────────────────
    work_items = []
    for i, session_dir in enumerate(sessions):
        episode_id = args.episode_start + i
        out_path   = os.path.join(args.out_dir, f"{os.path.basename(session_dir)}.hdf5")

        if os.path.exists(out_path) and not args.overwrite:
            logger.info("SKIP (exists): %s", out_path)
            continue

        work_items.append((
            session_dir,
            args.out_dir,
            episode_id,
            args.jpeg_quality,
            args.encode_threads,
            effective_cam_roles(session_dir),
            effective_xense_roles(session_dir),
            tag_bytes,
        ))

    if not work_items:
        logger.info("Nothing to convert.")
        return

    logger.info(
        "Converting %d session(s) with %d worker process(es)…",
        len(work_items), min(args.workers, len(work_items)),
    )

    # ── run parallel pool ────────────────────────────────────────────────────
    n_workers = min(args.workers, len(work_items))
    if n_workers == 1:
        results = [process_session_worker(w) for w in work_items]
    else:
        with mp.Pool(processes=n_workers) as pool:
            results = pool.map(process_session_worker, work_items)

    # ── summary ──────────────────────────────────────────────────────────────
    ok = sum(1 for r in results if r.startswith("OK"))
    sk = sum(1 for r in results if r.startswith("SKIP"))
    er = sum(1 for r in results if r.startswith("ERROR"))
    for r in results:
        if r.startswith("ERROR"):
            logger.error(r)
        else:
            logger.info(r)
    logger.info("Done — OK: %d  SKIP: %d  ERROR: %d", ok, sk, er)


if __name__ == "__main__":
    main()
