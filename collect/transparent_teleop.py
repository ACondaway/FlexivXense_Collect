#!/usr/bin/env python3
"""
Thin wrapper around Flexiv TDK TransparentCartesianTeleopLAN for one robot pair.

The setup sequence mirrors the official example:
third_parties/flexiv_tdk/example_py/transparent_cartesian_teleop_lan.py

Exposes the same interface as CartesianTeleopPair so dual_collect.py needs
no structural changes — only import paths differ.
"""

from threading import Lock
from typing import Optional, Sequence, Tuple

import numpy as np

# pip install flexivtdk
import flexivtdk


TDK_TCP_POSE_ORDER = "[x, y, z, qw, qx, qy, qz]"
SAVED_TCP_POSE_ORDER = "[x, y, z, qx, qy, qz, qw]"


def tdk_pose_to_saved_xyzquat(tdk_pose: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Convert TDK/RDK pose order to the saved dataset pose order."""
    tdk_pose = np.asarray(tdk_pose, dtype=np.float64)
    xyz = tdk_pose[:3].copy()
    # TDK order: [x, y, z, qw, qx, qy, qz]  →  saved order: [qx, qy, qz, qw]
    quat_xyzw = np.array(
        [tdk_pose[4], tdk_pose[5], tdk_pose[6], tdk_pose[3]],
        dtype=np.float64,
    )
    return xyz, quat_xyzw


class TransparentCartesianTeleopPair:
    """
    Thin wrapper around flexivtdk.TransparentCartesianTeleopLAN for one robot pair.

    Drop-in replacement for CartesianTeleopPair: exposes identical methods so
    dual_collect.py works without structural changes.

    Leader  = first_sn  (master, the operator-side arm)
    Follower = second_sn (slave,  the task-side arm)
    """

    def __init__(
        self,
        first_sn: str,
        second_sn: str,
        robot_pair_idx: int = 0,
        network_interface_whitelist: Optional[Sequence[str]] = None,
    ) -> None:
        self.first_sn = first_sn
        self.second_sn = second_sn
        self.robot_pair_idx = robot_pair_idx
        self.lock = Lock()
        self.started = False
        self.engaged = False

        robot_pairs = [(self.first_sn, self.second_sn)]
        lan_ips = list(network_interface_whitelist) if network_interface_whitelist else []
        self.cart_teleop = flexivtdk.TransparentCartesianTeleopLAN(robot_pairs, lan_ips)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def init(self) -> None:
        """Run the official TDK initialization sequence (Init → Start)."""
        with self.lock:
            self.cart_teleop.Init()
            self.cart_teleop.Start()
            self.started = True

    def home_robots(
        self,
        first_tool_name: str = "tool2",
        second_tool_name: str = "xense",
    ) -> None:
        """Home both robots, then restore TDK control.

        Sequence: Stop TDK → home first robot → home second robot → Start TDK.
        Must be called while NOT engaged (i.e. before activate(True)).
        """
        import sys
        import os
        sys.path.insert(0, os.path.dirname(__file__))
        from homing import home_robot

        with self.lock:
            if self.started:
                self.cart_teleop.Stop()
                self.started = False

        home_robot(self.first_sn, first_tool_name)
        home_robot(self.second_sn, second_tool_name)

        with self.lock:
            self.cart_teleop.Start()
            self.started = True

    # ------------------------------------------------------------------
    # Teleoperation control
    # ------------------------------------------------------------------

    def activate(self, activated: bool) -> None:
        """Engage or disengage teleoperation for this robot pair."""
        with self.lock:
            self.cart_teleop.Engage(self.robot_pair_idx, activated)
            self.engaged = activated

    def sync_null_space_postures(self) -> np.ndarray:
        """Sync follower null-space posture to the leader's current joint angles.

        Skipped silently when the TDK pair is in a stopped state (e.g. after
        a robot fault) so the calling loop does not crash before any_fault()
        detects the condition on its next iteration.
        """
        with self.lock:
            if not self.started or self.cart_teleop.stopped(self.robot_pair_idx):
                return np.array([])
            leader_q = self.cart_teleop.robot_states(self.robot_pair_idx)[0].q
            self.cart_teleop.SetLeaderNullSpacePosture(self.robot_pair_idx, leader_q)
            self.cart_teleop.SetFollowerNullSpacePosture(self.robot_pair_idx, leader_q)
        return leader_q

    # ------------------------------------------------------------------
    # State reading
    # ------------------------------------------------------------------

    def read_states(self):
        """Return (leader_state, follower_state) from TDK."""
        with self.lock:
            return self.cart_teleop.robot_states(self.robot_pair_idx)

    def read_master_state(self):
        """Return the leader (master) robot state."""
        return self.read_states()[0]

    def read_slave_state(self):
        """Return the follower (slave) robot state."""
        return self.read_states()[1]

    def read_slave_tcp_pose_and_joints(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return follower TCP pose in TDK order and follower joint angles."""
        slave_state = self.read_slave_state()
        tcp_pose = np.asarray(slave_state.tcp_pose, dtype=np.float64)
        joint_angles = np.asarray(slave_state.q, dtype=np.float64)
        return tcp_pose, joint_angles

    def read_slave_saved_xyzquat_and_joints(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return follower TCP pose in dataset order and follower joint angles."""
        tcp_pose, joint_angles = self.read_slave_tcp_pose_and_joints()
        tcp_xyz, tcp_quat_xyzw = tdk_pose_to_saved_xyzquat(tcp_pose)
        return tcp_xyz, tcp_quat_xyzw, joint_angles

    def read_both_saved_xyzquat_and_joints(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Read leader and follower TCP poses + joint angles in a single locked call.

        Returns
        -------
        leader_xyz, leader_quat_xyzw, leader_joints,
        follower_xyz, follower_quat_xyzw, follower_joints
        All arrays are float64.  Quaternion order: [qx, qy, qz, qw].
        """
        leader_state, follower_state = self.read_states()

        leader_tcp = np.asarray(leader_state.tcp_pose, dtype=np.float64)
        leader_joints = np.asarray(leader_state.q, dtype=np.float64)
        leader_xyz, leader_quat = tdk_pose_to_saved_xyzquat(leader_tcp)

        follower_tcp = np.asarray(follower_state.tcp_pose, dtype=np.float64)
        follower_joints = np.asarray(follower_state.q, dtype=np.float64)
        follower_xyz, follower_quat = tdk_pose_to_saved_xyzquat(follower_tcp)

        return (
            leader_xyz, leader_quat, leader_joints,
            follower_xyz, follower_quat, follower_joints,
        )

    # ------------------------------------------------------------------
    # Fault handling
    # ------------------------------------------------------------------

    def fault(self):
        """Return fault state of this robot pair."""
        with self.lock:
            return self.cart_teleop.fault(self.robot_pair_idx)

    def any_fault(self) -> bool:
        """Return True if any connected robot is in fault state."""
        with self.lock:
            return self.cart_teleop.any_fault()

    def clear_fault(self, timeout_sec: int = 30):
        """Try to clear minor or critical faults."""
        with self.lock:
            return self.cart_teleop.ClearFault(timeout_sec)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Disengage this pair and stop the TDK control loop."""
        with self.lock:
            if self.engaged:
                try:
                    self.cart_teleop.Engage(self.robot_pair_idx, False)
                except Exception:
                    pass  # TDK may have already stopped due to a fault
                self.engaged = False
            if self.started:
                self.cart_teleop.Stop()
                self.started = False

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        try:
            self.init()
        except Exception:
            self.stop()
            raise
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()


class TeleopSlaveStateReader:
    """Small adapter used by the data collection thread."""

    def __init__(self, teleop_pair: TransparentCartesianTeleopPair) -> None:
        self.teleop_pair = teleop_pair

    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.teleop_pair.read_slave_tcp_pose_and_joints()

    def read_saved_xyzquat(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.teleop_pair.read_slave_saved_xyzquat_and_joints()

    def read_both_saved_xyzquat(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Read leader + follower TCP pose and joint angles in one locked call."""
        return self.teleop_pair.read_both_saved_xyzquat_and_joints()
