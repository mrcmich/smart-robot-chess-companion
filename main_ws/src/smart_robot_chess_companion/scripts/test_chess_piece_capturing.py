#! /usr/bin/env python

from smart_robot_chess_companion import pick_and_place, pick_and_place_config
from ur_control import arm, constants
import rospy

def test_chess_piece_capturing(robot_arm):
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
        final_cell = pick_and_place_config.MAPPING_N_CAPTURED_CHESS_PIECES_CELL[n_captured_chess_pieces]
        print(f'Capturing chess piece {chess_piece_type} in cell {starting_cell}...')
        pick_and_place.move_chess_piece(
            robot_arm, 
            chess_piece_type, 
            starting_cell, 
            final_cell, 
            grasp_positions_z_dict=pick_and_place_config.DEFAULT_GRASP_POSITIONS_Z_DICT,
            grasp_gripper_positions_dict=pick_and_place_config.DEFAULT_GRASP_GRIPPER_POSITIONS_DICT, 
        )
        n_captured_chess_pieces += 1

def main():
    rospy.init_node('ur3e_script_control', log_level=rospy.DEBUG)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_arm.set_joint_positions(position=pick_and_place_config.REST_ROBOT_ARM_CONFIGURATION, wait=True, t=0.5)
    robot_arm.gripper.command(pick_and_place_config.OPEN_GRIPPER_POSITION)
    test_chess_piece_capturing(robot_arm)
    print('All tests completed!')
    
if __name__ == "__main__":
    main()
    
