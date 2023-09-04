#! /usr/bin/env python

from smart_robot_chess_companion.pick_and_place import pick_and_place_chess_piece
from smart_robot_chess_companion.constants import STARTING_ROBOT_ARM_CONFIGURATION, GRIPPER_OPEN_POSITION
from ur_control import arm, constants
from ur_control.constants import GENERIC_GRIPPER
import rospy
import timeit

def main():
    rospy.init_node('ur3e_script_control')
    global robot_arm
    robot_arm = arm.Arm(ft_sensor=True, gripper=GENERIC_GRIPPER)
    robot_arm.set_joint_positions(position=STARTING_ROBOT_ARM_CONFIGURATION, wait=True, t=1.0)
    robot_arm.gripper.command(GRIPPER_OPEN_POSITION)

    real_start_time = timeit.default_timer()
    ros_start_time = rospy.get_time()

    pick_and_place_chess_piece(robot_arm, chess_piece_type='king', starting_cell='e1')

    print("real time", round(timeit.default_timer() - real_start_time, 3))
    print("ros time", round(rospy.get_time() - ros_start_time, 3))


if __name__ == "__main__":
    main()
