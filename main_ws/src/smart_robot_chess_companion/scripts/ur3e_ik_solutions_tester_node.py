#! /usr/bin/env python

import smart_robot_chess_companion.pick_and_place.config as config
import smart_robot_chess_companion.pick_and_place.utils as utils
from smart_robot_chess_companion.pick_and_place.pick_and_place import RobotChessCompanion
from ur_control import arm, constants
import rospy

def ur3e_ik_solutions_tester_node():
    rospy.init_node('ur3e_ik_solutions_tester_node', log_level=rospy.DEBUG)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_chess_companion = RobotChessCompanion(
        robot_arm,
        config.REST_ROBOT_ARM_CONFIGURATION,
        config.RIGHT_ARM_ROBOT_ARM_CONFIGURATION,
        config.LEFT_ARM_ROBOT_ARM_CONFIGURATION,
        config.DEFAULT_GRASP_CONFIGURATIONS_DICT,
        config.OPEN_GRIPPER_POSITION
    )
    
    for chess_piece_type in ['king', 'pawn', 'knight', 'bishop', 'queen', 'rook']:
        for column in list('wyxabcdefghjkl'):
            for row in range(1, 10):
                cell = column + str(row)

                if not utils.is_valid_cell(cell):
                    continue

                rospy.logdebug(f'Searching IK solutions for chess piece {chess_piece_type} in cell {cell}...')
                robot_chess_companion.move_chess_piece(
                    chess_piece_type, 
                    starting_cell=cell, 
                    final_cell=cell,  
                    grasping=False, 
                )
                rospy.sleep(0.5)
                
    rospy.logdebug('All tests completed!')

if __name__ == '__main__':
    try:
        ur3e_ik_solutions_tester_node()
    except rospy.ROSInterruptException:
        pass
