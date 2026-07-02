#!/usr/bin/env python3
"""
Thin wrapper around Flexiv TDK Cartesian-space robot-robot teleoperation under LAN.

The setup sequence intentionally mirrors the official example:
Ref/flexiv_tdk/example_py/cartesian_teleop_under_lan_auto.py
"""

from threading import Lock
from typing import Optional, Sequence, Tuple

import numpy as np

# pip install flexivtdk
import flexivtdk


# Constants copied from the official TDK example
SHAPED_CART_INERTIA = [60.0, 60.0, 60.0, 20.0, 20.0, 20.0]
CART_STIFFNESS_RATIO = [0.1, 0.1, 0.1, 0.005, 0.005, 0.005]
CART_DAMPING_RATIO = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]

TDK_TCP_POSE_ORDER = "[x, y, z, qw, qx, qy, qz]"
SAVED_TCP_POSE_ORDER = "[x, y, z, qx, qy, qz, qw]"


def tdk_pose_to_saved_xyzquat(tdk_pose: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Convert TDK/RDK pose order to the saved dataset pose order."""
    tdk_pose = np.asarray(tdk_pose, dtype=np.float64)

    xyz = tdk_pose[:3].copy()
    quat_xyzw = np.array(
        [tdk_pose[4], tdk_pose[5], tdk_pose[6], tdk_pose[3]],
        dtype=np.float64,
    )
    return xyz, quat_xyzw


class CartesianTeleopPair:
    """Thin wrapper around flexivtdk.CartesianTeleopLAN for one robot pair."""

    def __init__(
        self,
        first_sn: str,
        second_sn: str,
        robot_pair_idx: int = 0,
        network_interface_whitelist: Optional[Sequence[str]] = None,
        shaped_cart_inertia: Sequence[float] = SHAPED_CART_INERTIA,
        cart_stiffness_ratio: Sequence[float] = CART_STIFFNESS_RATIO,
        cart_damping_ratio: Sequence[float] = CART_DAMPING_RATIO,
    ) -> None:
        self.first_sn = first_sn
        self.second_sn = second_sn
        self.robot_pair_idx = robot_pair_idx
        self.shaped_cart_inertia = list(shaped_cart_inertia)
        self.cart_stiffness_ratio = list(cart_stiffness_ratio)
        self.cart_damping_ratio = list(cart_damping_ratio)
        self.lock = Lock()
        self.started = False
        self.activated = False

        robot_pairs = [(self.first_sn, self.second_sn)]
        if network_interface_whitelist is None:
            self.cart_teleop = flexivtdk.CartesianTeleopLAN(robot_pairs)
        else:
            self.cart_teleop = flexivtdk.CartesianTeleopLAN(
                robot_pairs,
                list(network_interface_whitelist),
            )

    def init(self) -> None:
        """Run the official TDK initialization sequence."""
        # Run initialization sequence
        with self.lock:
            self.cart_teleop.Init()

        self.sync_pose()
        self.set_inertia_shaping()
        self.start()
        self.set_cartesian_impedance()

    def sync_pose(self):
        """Sync pose: first robot stays still, second robot moves to its TCP pose."""
        with self.lock:
            first_robot_state, second_robot_state = self.cart_teleop.robot_states(
                self.robot_pair_idx
            )
            self.cart_teleop.SyncPose(
                self.robot_pair_idx,
                first_robot_state.tcp_pose,
            )
        return first_robot_state, second_robot_state

    def set_inertia_shaping(self) -> None:
        """Enable inertia shaping for all Cartesian axes."""
        shaped_cart_inertia = []
        for i in range(flexivtdk.kCartDoF):
            shaped_cart_inertia.append((True, self.shaped_cart_inertia[i]))

        with self.lock:
            self.cart_teleop.SetInertiaShaping(
                self.robot_pair_idx,
                shaped_cart_inertia,
            )

    def start(self) -> None:
        """Start the TDK teleoperation control loop."""
        with self.lock:
            self.cart_teleop.Start()
            self.started = True

    def set_cartesian_impedance(self) -> None:
        """Set Cartesian impedance properties."""
        with self.lock:
            self.cart_teleop.SetCartesianImpedance(
                self.robot_pair_idx,
                self.cart_stiffness_ratio,
                self.cart_damping_ratio,
            )

    def activate(self, activated: bool) -> None:
        """Activate or deactivate teleoperation for this robot pair."""
        with self.lock:
            self.cart_teleop.Activate(self.robot_pair_idx, activated)
            self.activated = activated

    def read_states(self):
        """Return value copies of the first and second robot states."""
        with self.lock:
            return self.cart_teleop.robot_states(self.robot_pair_idx)

    def read_master_state(self):
        """Return the first robot state."""
        return self.read_states()[0]

    def read_slave_state(self):
        """Return the second robot state."""
        return self.read_states()[1]

    def read_slave_tcp_pose_and_joints(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return slave TCP pose in TDK order and slave joint angles."""
        slave_state = self.read_slave_state()
        tcp_pose = np.asarray(slave_state.tcp_pose, dtype=np.float64)
        joint_angles = np.asarray(slave_state.q, dtype=np.float64)
        return tcp_pose, joint_angles

    def read_slave_saved_xyzquat_and_joints(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return slave TCP pose in dataset order and slave joint angles."""
        tcp_pose, joint_angles = self.read_slave_tcp_pose_and_joints()
        tcp_xyz, tcp_quat_xyzw = tdk_pose_to_saved_xyzquat(tcp_pose)
        return tcp_xyz, tcp_quat_xyzw, joint_angles

    def read_both_saved_xyzquat_and_joints(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Read master and slave TCP poses + joint angles in a single locked call.

        Returns
        -------
        master_xyz, master_quat_xyzw, master_joints,
        slave_xyz,  slave_quat_xyzw,  slave_joints
        All arrays are float64.  Quaternion order: [qx, qy, qz, qw].
        """
        master_state, slave_state = self.read_states()

        master_tcp = np.asarray(master_state.tcp_pose, dtype=np.float64)
        master_joints = np.asarray(master_state.q, dtype=np.float64)
        master_xyz, master_quat = tdk_pose_to_saved_xyzquat(master_tcp)

        slave_tcp = np.asarray(slave_state.tcp_pose, dtype=np.float64)
        slave_joints = np.asarray(slave_state.q, dtype=np.float64)
        slave_xyz, slave_quat = tdk_pose_to_saved_xyzquat(slave_tcp)

        return master_xyz, master_quat, master_joints, slave_xyz, slave_quat, slave_joints

    def sync_null_space_postures(self):
        """Sync null-space posture of the second robot to that of the first."""
        with self.lock:
            first_robot_q = self.cart_teleop.robot_states(self.robot_pair_idx)[0].q
            null_space_postures = (first_robot_q, first_robot_q)
            self.cart_teleop.SetNullSpacePostures(
                self.robot_pair_idx,
                null_space_postures,
            )
        return null_space_postures

    def fault(self):
        """Return fault state of this robot pair."""
        with self.lock:
            return self.cart_teleop.fault(self.robot_pair_idx)

    def any_fault(self) -> bool:
        """Return whether any connected robot is in fault state."""
        with self.lock:
            return self.cart_teleop.any_fault()

    def clear_fault(self, timeout_sec: int = 30):
        """Try to clear minor or critical faults."""
        with self.lock:
            return self.cart_teleop.ClearFault(timeout_sec)

    def stop(self) -> None:
        """Deactivate this pair and stop the TDK control loop."""
        with self.lock:
            if self.activated:
                self.cart_teleop.Activate(self.robot_pair_idx, False)
                self.activated = False
            if self.started:
                self.cart_teleop.Stop()
                self.started = False

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

    def __init__(self, teleop_pair: CartesianTeleopPair) -> None:
        self.teleop_pair = teleop_pair

    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.teleop_pair.read_slave_tcp_pose_and_joints()

    def read_saved_xyzquat(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.teleop_pair.read_slave_saved_xyzquat_and_joints()

    def read_both_saved_xyzquat(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Read master + slave TCP pose and joint angles in one locked call."""
        return self.teleop_pair.read_both_saved_xyzquat_and_joints()
