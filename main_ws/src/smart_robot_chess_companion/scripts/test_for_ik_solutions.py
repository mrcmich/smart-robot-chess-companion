#! /usr/bin/env python

from smart_robot_chess_companion import pick_and_place, pick_and_place_config, pick_and_place_utils
from ur_control import arm, constants
import rospy

def test_for_ik_solutions_single_cell(robot_arm, chess_piece_type, cell):
    print(f'Searching IK solutions for chess piece {chess_piece_type} in cell {cell}...')
    pick_and_place.move_chess_piece(
        robot_arm, 
        chess_piece_type, 
        starting_cell=cell, 
        final_cell=cell, 
        grasp_positions_z_dict=pick_and_place_config.DEFAULT_GRASP_POSITIONS_Z_DICT,
        grasp_gripper_positions_dict=pick_and_place_config.DEFAULT_GRASP_GRIPPER_POSITIONS_DICT, 
        grasping=False, 
    )

def test_for_ik_solutions(robot_arm):
    for column in list('wyxabcdefghjkl'):
        for row in range(1, 10):
            cell = column + str(row)

            if not pick_and_place_utils.is_valid_cell(cell):
                continue

            for chess_piece_type in ['king', 'pawn', 'knight', 'bishop', 'queen', 'rook']:
                test_for_ik_solutions_single_cell(robot_arm, chess_piece_type, cell)
                rospy.sleep(0.5)

def main():
    rospy.init_node('ur3e_script_control', log_level=rospy.DEBUG)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_arm.set_joint_positions(position=pick_and_place_config.REST_ROBOT_ARM_CONFIGURATION, wait=True, t=0.5)
    robot_arm.gripper.command(pick_and_place_config.OPEN_GRIPPER_POSITION)
    test_for_ik_solutions(robot_arm)
    print('All tests completed!')
    
if __name__ == "__main__":
    main()
    
