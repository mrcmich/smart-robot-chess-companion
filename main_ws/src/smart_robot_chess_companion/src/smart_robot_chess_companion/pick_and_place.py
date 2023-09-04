import numpy as np
from gazebo_msgs.srv import GetModelState
from ur_control import transformations
import rospy
from smart_robot_chess_companion.constants import STARTING_ROBOT_ARM_CONFIGURATION, GRIPPER_OPEN_POSITION

TABLE_HEIGHT = 0.775
CHESS_BOARD_SIZE = (0.440, 0.440, 0.03)
GRIPPER_POSITION_Z_MARGIN = 0.05
Z_DISTANCE_ROBOT_WRIST3_GRIPPER_FINGER_FRAMES = 0.107
GRIPPER_POSITION_Z_MARGIN = 0.05
MIN_GRIPPER_POSITION_Z = TABLE_HEIGHT + CHESS_BOARD_SIZE[-1] + Z_DISTANCE_ROBOT_WRIST3_GRIPPER_FINGER_FRAMES + GRIPPER_POSITION_Z_MARGIN
CHESS_BOARD_LINE_POSITIONS = np.array([-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5]) * CHESS_BOARD_SIZE[0] / 10
CAPTURED_PIECE_POSITION = np.array([0., -0.337, MIN_GRIPPER_POSITION_Z + 0.1])

TRANSFORMATION_ROBOT_BASE_FRAME_WORLD_FRAME = np.array([
    [0., -1., 0.,  0.],
    [1.,  0., 0.,  0.335],
    [0.,  0., 1., -TABLE_HEIGHT],
    [0.,  0., 0.,  1.]
])

MODEL_NAMES_DICT = {
    'king': [
        'king_white', 
        'king_black'
    ],
    'pawn': [
        'pawn_white_1', 'pawn_white_2', 'pawn_white_3', 'pawn_white_4', 'pawn_white_5', 'pawn_white_6', 'pawn_white_7', 'pawn_white_8', 
        'pawn_black_1', 'pawn_black_2', 'pawn_black_3', 'pawn_black_4', 'pawn_black_5', 'pawn_black_6', 'pawn_black_7', 'pawn_black_8'
    ],
    'queen': [
        'queen_white', 
        'queen_black'
    ],
    'rook': [
        'rook_white', 
        'rook_black'
    ],
    'bishop': [
        'bishop_white', 
        'bishop_black'
    ],
    'knight': [
        'knight_white', 
        'knight_black'
    ],
}

def get_chess_piece_grasp_configuration(chess_piece_type):
    if chess_piece_type == 'king':
        return (0.02, 0.021)
    else:
        None

def get_chess_piece_model_name(chess_piece_type, chess_piece_position=[0., 0.]):
    model_state_service_proxy = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    model_name = None
    min_squared_distance = -1
    
    for candidate_model_name in MODEL_NAMES_DICT[chess_piece_type]:
        candidate_model_state = model_state_service_proxy(candidate_model_name, 'link')
        candidate_model_position = candidate_model_state.pose.position.x, candidate_model_state.pose.position.y
        squared_distance = (chess_piece_position[0] - candidate_model_position[0]) ** 2 + (chess_piece_position[1] - candidate_model_position[1]) ** 2
        
        if squared_distance == 0:
            return candidate_model_name
        
        if min_squared_distance == -1 or squared_distance < min_squared_distance:
            model_name = candidate_model_name
            min_squared_distance = squared_distance
    
    return model_name

def get_chess_piece_position(chess_piece_cell):
    chess_piece_row = int(chess_piece_cell[1])
    chess_piece_column = chess_piece_cell[0].lower()

    if chess_piece_column not in list('abcdefgh'):
        return None
    
    chess_piece_position_x = CHESS_BOARD_LINE_POSITIONS[8 - chess_piece_row]
    chess_piece_position_y = CHESS_BOARD_LINE_POSITIONS[ord(chess_piece_column) - 97]

    return np.array([chess_piece_position_x, chess_piece_position_y])

def move_end_effector(robot_arm, target_position_world, target_orientation_eulerzyz=[0., 3.14, -3.14], wait=True, duration=1.0):
    target_position_base = np.dot(TRANSFORMATION_ROBOT_BASE_FRAME_WORLD_FRAME, np.append(target_position_world, [1.]))[:-1]
    
    target_pose_position = np.append(target_position_base, [0., 0., 0., 0.])
    target_pose_orientation = transformations.euler_matrix(
        target_orientation_eulerzyz[0], target_orientation_eulerzyz[1], target_orientation_eulerzyz[2], axes='rzyz')
    target_pose = transformations.pose_quaternion_from_matrix(target_pose_orientation) + target_pose_position
    robot_arm.set_target_pose(pose=target_pose, wait=wait, t=duration)

def pick_and_place_chess_piece(robot_arm, chess_piece_type, starting_cell, final_cell=None, 
                               target_orientation_eulerzyz=[0., 3.14, -3.14], wait=True, duration=1.0):
    
    grasp_position_z, gripper_grasp_position = get_chess_piece_grasp_configuration(chess_piece_type)
    starting_piece_position_x, starting_piece_position_y = get_chess_piece_position(starting_cell)
    piece_model_name = get_chess_piece_model_name(chess_piece_type, [starting_piece_position_x, starting_piece_position_y])
    grasp_setup_position = np.array([starting_piece_position_x, starting_piece_position_y, MIN_GRIPPER_POSITION_Z + 0.1])
    grasp_position = np.array([starting_piece_position_x, starting_piece_position_y, MIN_GRIPPER_POSITION_Z + grasp_position_z])
    chess_piece_link_name = f'{piece_model_name}::link'

    # pick chess piece
    move_end_effector(robot_arm, grasp_setup_position, target_orientation_eulerzyz, wait, duration)
    move_end_effector(robot_arm, grasp_position, target_orientation_eulerzyz, wait, duration)
    robot_arm.gripper.command(gripper_grasp_position)
    robot_arm.gripper.grab(link_name=chess_piece_link_name)
   
    # place chess piece
    move_end_effector(robot_arm, grasp_setup_position, target_orientation_eulerzyz, wait, duration)
    if final_cell is None:
        move_end_effector(robot_arm, CAPTURED_PIECE_POSITION, target_orientation_eulerzyz, wait)
        robot_arm.gripper.command(GRIPPER_OPEN_POSITION) 
        robot_arm.gripper.release(link_name=chess_piece_link_name)
    else:
        # TODO: caso in cui final_cell è una cella valida
        print('Caso non implementato: final_cell not None')
    
    robot_arm.set_joint_positions(position=STARTING_ROBOT_ARM_CONFIGURATION, wait=wait, t=duration)
