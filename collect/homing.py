'''
usage: python homing.py -id <1 or 2>
'''
from r3kit.devices.robot.flexiv.rizon import Rizon
import argparse

def parse_args():
    parser = argparse.ArgumentParser(
        description="Choose which robot to home",
    )
    parser.add_argument("id", type=int, required=True, help="1 refers to master robot, 2 refers to slave robot")

def main():
    args = parse_args()
    if args.id == 1:
        robot = Rizon(id='Rizon4s-063652', gripper=False, name='Rizon4s')
    elif args.id == 2:
        robot = Rizon(id='Rizon4s-063586', gripper=False, name='Rizon4s')
    else:
        raise ValueError("Invalid robot ID")
    robot.homing()
