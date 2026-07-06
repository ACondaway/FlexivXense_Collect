#!/usr/bin/env python3
"""
Dual-arm teleoperation data collection entrypoint.

This script only orchestrates devices and threads. TDK teleoperation lives in
dual_teleop.py, and data saving utilities live in dual_collect_utils.py.
"""

import argparse
import logging
import select
import sys
import threading
import time
import flexivrdk # this must be imported
from datetime import datetime

DEFAULT_FPS = 30

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("DualCollect")


def _read_key_nonblocking():
    """Read one key from stdin without blocking, return None if no key available."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        return sys.stdin.read(1)
    return None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Dual-arm Cartesian teleoperation data collection under LAN",
    )
    parser.add_argument("-1", "--first-sn", required=True, help="Master robot serial number")
    parser.add_argument("-2", "--second-sn", required=True, help="Slave robot serial number")
    parser.add_argument("--slave-gripper-id", default=None, help="Slave Xense gripper ID")
    parser.add_argument("--save-root", required=True, help="Root directory for collected data")
    parser.add_argument("--session-name", default=None, help="Optional session directory name")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Collection FPS")
    parser.add_argument(
        "--use-gripper",
        type=parse_bool,
        default=True,
        help="Whether to initialize, sync, and collect slave gripper width",
    )
    parser.add_argument(
        "--network-interface",
        action="append",
        default=None,
        help="Optional LAN interface whitelist IPv4 address. Can be repeated.",
    )
    parser.add_argument("--gripper-eps", type=float, default=1e-4, help="Gripper sync threshold")
    parser.add_argument("--gripper-wait-time", type=float, default=0.1, help="Delay after gripper move")
    parser.add_argument("--null-space-period", type=float, default=0.1, help="Main loop period")
    parser.add_argument("--angler-id", default="/dev/ttyUSB0", help="Master Angler serial port")
    parser.add_argument("--angler-index", type=int, default=1, help="Master Angler encoder index")
    parser.add_argument("--angler-baudrate", type=int, default=1000000, help="Master Angler baudrate")
    parser.add_argument("--angler-gap", type=float, default=-1.0, help="Master Angler read gap")
    parser.add_argument("--angler-strict", type=parse_bool, default=True, help="Whether Angler uses strict CRC retry")
    parser.add_argument("--angler-open-angle", type=float, default=51.68, help="Angle when slave gripper should be open")
    parser.add_argument("--angler-close-angle", type=float, default=16.61, help="Angle when slave gripper should be closed")
    parser.add_argument("--slave-open-width", type=float, default=0.085, help="Slave gripper open width in meters")
    parser.add_argument("--slave-close-width", type=float, default=0.0, help="Slave gripper closed width in meters")
    parser.add_argument("--num-writers", type=int, default=4, help="Number of parallel disk-writer subprocesses")
    parser.add_argument("--png-level", type=int, default=1, help="PNG compression level 0–9 (0=none, 9=max)")
    parser.add_argument("--xense-left-id", default=None, help="Left end-effector Xense camera device ID")
    parser.add_argument("--xense-left-mac", default=None, help="Left end-effector Xense camera MAC address")
    parser.add_argument("--xense-right-id", default=None, help="Right end-effector Xense camera device ID")
    parser.add_argument("--xense-right-mac", default=None, help="Right end-effector Xense camera MAC address")
    parser.add_argument("--xense-fps", type=int, default=50, help="Xense tactile sensor sampling rate Hz")
    args = parser.parse_args()
    if args.use_gripper and not args.slave_gripper_id:
        parser.error("--use-gripper true requires --slave-gripper-id")
    if args.use_gripper and args.angler_open_angle == args.angler_close_angle:
        parser.error("--angler-open-angle and --angler-close-angle must be different")
    return args


def parse_bool(value):
    if isinstance(value, bool):
        return value

    value = value.lower()
    if value in ("true", "1", "yes", "y"):
        return True
    if value in ("false", "0", "no", "n"):
        return False
    raise argparse.ArgumentTypeError("Expected true or false")


def build_metadata(args, camera_serials, tdk_tcp_pose_order, saved_tcp_pose_order):
    metadata = vars(args).copy()
    metadata.update(
        {
            "camera_serials": camera_serials,
            "recorded_robot": "second",
            "tcp_pose_source": "TransparentCartesianTeleopLAN.robot_states()[1].tcp_pose",
            "tdk_tcp_pose_order": tdk_tcp_pose_order,
            "saved_tcp_pose_order": saved_tcp_pose_order,
            "master_gripper_width_source": (
                "disabled"
                if not args.use_gripper
                else "Angler angle linear mapping"
            ),
            "slave_gripper_width_source": (
                "slave_gripper.read()" if args.use_gripper else "constant_zero"
            ),
        }
    )
    return metadata


def sync_gripper(master_gripper, slave_gripper, last_width, eps, wait_time):
    master_width = master_gripper.read()
    if last_width is None or abs(master_width - last_width) > eps:
        slave_gripper.move(master_width)
        last_width = master_width
        if wait_time > 0:
            time.sleep(wait_time)
    return last_width


def stop_collection(stop_event, collect_thread) -> None:
    if stop_event is not None:
        stop_event.set()
    if collect_thread is not None:
        collect_thread.join(timeout=2.0)


def start_recording(
    args,
    state_reader,
    slave_gripper,
    cameras,
    d415_cameras,
    tdk_tcp_pose_order,
    saved_tcp_pose_order,
    xense_sensors=None,
):
    from dual_collect_utils import collect_teleop_data_parallel, create_session_dirs, write_metadata

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    session_name = f"{args.session_name}_{timestamp}" if args.session_name else f"record_{timestamp}"

    session_dir = create_session_dirs(
        args.save_root,
        d415_cameras=d415_cameras,
        session_name=session_name,
        xense_sensors=xense_sensors,
    )
    write_metadata(
        session_dir,
        build_metadata(args, d415_cameras, tdk_tcp_pose_order, saved_tcp_pose_order),
    )

    stop_event = threading.Event()
    collect_thread = threading.Thread(
        target=collect_teleop_data_parallel,
        args=(
            state_reader,
            slave_gripper,
            cameras,
            session_dir,
            stop_event,
            args.fps,
            args.use_gripper,
            args.num_writers,
            args.png_level,
            100,            # status_period
            xense_sensors,
            args.xense_fps,
        ),
        daemon=True,
    )
    collect_thread.start()
    return session_dir, stop_event, collect_thread


def run_keyboard_loop(
    args,
    teleop_pair,
    state_reader,
    cameras,
    master_gripper,
    slave_gripper,
    d415_cameras,
    tdk_tcp_pose_order,
    saved_tcp_pose_order,
    gripper_eps,
    gripper_wait_time,
    null_space_period,
    use_gripper,
    xense_sensors=None,
) -> None:
    import termios
    import tty

    activated = False
    recording = False
    last_master_width = None
    stop_event = None
    collect_thread = None
    print(
        "Keyboard control enabled: press 'r' to start teleop, 's' to stop teleop, "
        "'c' to start recording, 'v' to stop recording, 'q' to quit"
    )

    old_term_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while not teleop_pair.any_fault():
            key = _read_key_nonblocking()
            if key == "r" and not activated:
                teleop_pair.activate(True)
                activated = True
                logger.info("Teleoperation activated by keyboard")
            elif key == "s" and activated:
                teleop_pair.activate(False)
                activated = False
                logger.info("Teleoperation deactivated by keyboard")
                time.sleep(1)
                logger.info("Homing robots after teleoperation ...")
                teleop_pair.home_robots()
            elif key == "c" and not recording:
                session_dir, stop_event, collect_thread = start_recording(
                    args,
                    state_reader,
                    slave_gripper,
                    cameras,
                    d415_cameras,
                    tdk_tcp_pose_order,
                    saved_tcp_pose_order,
                    xense_sensors=xense_sensors,
                )
                recording = True
                logger.info("Recording started: %s", session_dir)
            elif key == "v" and recording:
                stop_collection(stop_event, collect_thread)
                stop_event = None
                collect_thread = None
                recording = False
                logger.info("Recording stopped")
            elif key == "q":
                logger.info("Quit requested by keyboard")
                break

            if use_gripper:
                last_master_width = sync_gripper(
                    master_gripper,
                    slave_gripper,
                    last_master_width,
                    gripper_eps,
                    gripper_wait_time,
                )
            teleop_pair.sync_null_space_postures()
            time.sleep(null_space_period)
    finally:
        if recording:
            stop_collection(stop_event, collect_thread)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term_settings)
        if activated:
            try:
                teleop_pair.activate(False)
            except Exception as e:
                logger.warning("Could not disengage teleop during cleanup: %s", e)


def main() -> None:
    args = parse_args()

    from transparent_teleop import (
        SAVED_TCP_POSE_ORDER,
        TDK_TCP_POSE_ORDER,
        TransparentCartesianTeleopPair as CartesianTeleopPair,
        TeleopSlaveStateReader,
    )
    from dual_collect_utils import (
        D415_CAMERAS,
        init_cameras,
        init_angler_controller,
        init_xense,
        init_xense_camera,
    )

    try:
        with CartesianTeleopPair(
            args.first_sn,
            args.second_sn,
            network_interface_whitelist=args.network_interface,
        ) as teleop_pair:
            master_gripper = None
            slave_gripper = None
            if args.use_gripper:
                slave_gripper = init_xense(args.slave_gripper_id, "slave_xense")
                master_gripper = init_angler_controller(
                    encoder_id=args.angler_id,
                    index=args.angler_index,
                    baudrate=args.angler_baudrate,
                    gap=args.angler_gap,
                    strict=args.angler_strict,
                    open_angle=args.angler_open_angle,
                    close_angle=args.angler_close_angle,
                    open_width=args.slave_open_width,
                    close_width=args.slave_close_width,
                )

            xense_sensors = {}
            if args.xense_left_id:
                xense_sensors["xense_left"] = init_xense_camera(
                    args.xense_left_id, name="xense_left", fps=args.xense_fps,
                    mac_addr=args.xense_left_mac,
                )
            if args.xense_right_id:
                xense_sensors["xense_right"] = init_xense_camera(
                    args.xense_right_id, name="xense_right", fps=args.xense_fps,
                    mac_addr=args.xense_right_mac,
                )

            cameras = init_cameras(D415_CAMERAS, args.fps)
            state_reader = TeleopSlaveStateReader(teleop_pair)

            run_keyboard_loop(
                args,
                teleop_pair,
                state_reader,
                cameras,
                master_gripper,
                slave_gripper,
                D415_CAMERAS,
                TDK_TCP_POSE_ORDER,
                SAVED_TCP_POSE_ORDER,
                args.gripper_eps,
                args.gripper_wait_time,
                args.null_space_period,
                args.use_gripper,
                xense_sensors=xense_sensors or None,
            )
    except Exception as e:
        logger.error(str(e))
        sys.exit(1)


if __name__ == "__main__":
    main()
