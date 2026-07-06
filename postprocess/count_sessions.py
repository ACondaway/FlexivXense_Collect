#!/usr/bin/env python3
"""
Count raw data sessions under a root directory.

A valid session directory must contain both tcps/ and angles/ subdirectories
(same criterion used by convert_to_hdf5.py).

Usage:
    python3 postprocess/count_sessions.py --save-root /path/to/raw
"""

import argparse
import os


def is_session_dir(path: str) -> bool:
    return (
        os.path.isdir(path)
        and os.path.isdir(os.path.join(path, "tcps"))
        and os.path.isdir(os.path.join(path, "angles"))
    )


def main():
    p = argparse.ArgumentParser(description="Count collected raw data sessions")
    p.add_argument("--save-root", required=True, help="Root directory to scan")
    args = p.parse_args()

    root = args.save_root
    if not os.path.isdir(root):
        print(f"Directory not found: {root}")
        return

    sessions = sorted(
        name for name in os.listdir(root)
        if is_session_dir(os.path.join(root, name))
    )

    print(f"Root: {root}")
    print(f"Sessions found: {len(sessions)}")
    for i, name in enumerate(sessions):
        session_dir = os.path.join(root, name)
        n_frames = len([
            f for f in os.listdir(os.path.join(session_dir, "tcps"))
            if f.endswith(".npy")
        ])
        print(f"  [{i+1:>3}] {name}  ({n_frames} frames)")


if __name__ == "__main__":
    main()
