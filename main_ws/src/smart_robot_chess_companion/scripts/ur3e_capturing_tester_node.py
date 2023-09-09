#! /usr/bin/env python

import smart_robot_chess_companion.pick_and_place.config as config
from smart_robot_chess_companion.pick_and_place.pick_and_place import RobotChessCompanion
from ur_control import arm, constants
import rospy

def ur3e_capturing_tester_node():
    rospy.init_node('ur3e_capturing_tester_node', log_level=rospy.DEBUG)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_chess_companion = RobotChessCompanion(
        robot_arm,
        config.REST_ROBOT_ARM_CONFIGURATION,
        config.RIGHT_ARM_ROBOT_ARM_CONFIGURATION,
        config.LEFT_ARM_ROBOT_ARM_CONFIGURATION,
        config.DEFAULT_GRASP_CONFIGURATIONS_DICT,
        config.OPEN_GRIPPER_POSITION
    )
    n_captured_chess_pieces = 0
    cell_mapping = {
        'a1': 'rook',
        'b1': 'knight',
        'c1': 'bishop',
        'd1': 'queen',
        'e1': 'king',
        'f1': 'bishop',
        'g1': 'knight',
        'h1': 'rook',
        'a2': 'pawn',
        'b2': 'pawn',
        'c2': 'pawn',
        'd2': 'pawn',
        'e2': 'pawn',
        'f2': 'pawn',
        'g2': 'pawn',
        'h2': 'pawn',
        'a8': 'rook',
        'b8': 'knight',
        'c8': 'bishop',
        'd8': 'queen',
        'e8': 'king',
        'f8': 'bishop',
        'g8': 'knight',
        'h8': 'rook',
        'a7': 'pawn',
        'b7': 'pawn',
        'c7': 'pawn',
        'd7': 'pawn',
        'e7': 'pawn',
        'f7': 'pawn',
        'g7': 'pawn',
        'h7': 'pawn',
    }
    
    for starting_cell in cell_mapping:
        chess_piece_type = cell_mapping[starting_cell]
        final_cell = config.MAPPING_N_CAPTURED_CHESS_PIECES_CELL[n_captured_chess_pieces]
        rospy.logdebug(f'n_captured_chess_pieces={n_captured_chess_pieces}')
        rospy.logdebug(f'Capturing chess piece {chess_piece_type} in cell {starting_cell}...')
        robot_chess_companion.move_chess_piece(
            chess_piece_type, 
            starting_cell, 
            final_cell
        )
        n_captured_chess_pieces += 1
        rospy.sleep(0.5)

    rospy.logdebug('All tests completed!')

if __name__ == '__main__':
    try:
        ur3e_capturing_tester_node()
    except rospy.ROSInterruptException:
        pass
    
