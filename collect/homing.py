"""
Usage:
    python homing.py -id <1 or 2>
"""
import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        description="Choose which robot to home",
    )
    parser.add_argument(
        "-id",
        "--id",
        dest="robot_id",
        type=int,
        choices=[1, 2],
        required=True,
        help="1 refers to master robot, 2 refers to slave robot",
    )
    return parser.parse_args()


def main():
    from r3kit.devices.robot.flexiv.rizon import Rizon

    args = parse_args()
    if args.robot_id == 1:
        robot_sn = "Rizon4s-063652"
        print(f"Homing robot {args.robot_id}: {robot_sn}")
        robot = Rizon(id=robot_sn, gripper=False, name="Rizon4s", tool_name="tool2")
    elif args.robot_id == 2:
        robot_sn = "Rizon4s-063586"
        print(f"Homing robot {args.robot_id}: {robot_sn}")
        robot = Rizon(id=robot_sn, gripper=False, name="Rizon4s", tool_name="xense")
    else:
        raise ValueError("Invalid robot ID")

    robot.motion_mode("joint")
    robot.homing()
    robot.motion_mode("primitive")
    print("Homing command finished")


if __name__ == "__main__":
    main()

