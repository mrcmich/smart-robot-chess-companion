import numpy as np
from ur_control import transformations
import math
from smart_robot_chess_companion import pick_and_place_config, pick_and_place_utils

#TODO da testare
def compute_captured_chess_piece_position(n_chess_pieces_captured):
    cell_size = pick_and_place_config.CHESS_BOARD_SIZE[0] / 10
    first_captured_chess_piece_position_x, first_captured_chess_piece_position_y = pick_and_place_config.CAPTURED_PIECES_FIRST_POSITION

    if n_chess_pieces_captured == 0:
        (i, j) = (0, 0)
    elif n_chess_pieces_captured == 11:
        (i, j) = (0, n_chess_pieces_captured)
    else:
        i = n_chess_pieces_captured // 11
        j = n_chess_pieces_captured * (0 <= n_chess_pieces_captured <= 11) + \
            (n_chess_pieces_captured - 11) * (12 <= n_chess_pieces_captured <= 21) + \
            (n_chess_pieces_captured - 21) * (22 <= n_chess_pieces_captured < 32)
    
    captured_chess_piece_position_x = first_captured_chess_piece_position_x + cell_size * i
    captured_chess_piece_position_y = first_captured_chess_piece_position_y + cell_size * j if j <= 5 else \
                                      -(first_captured_chess_piece_position_y + cell_size * j)
    
    return (captured_chess_piece_position_x, captured_chess_piece_position_y)

# ee_position_world:   position of end effector w.r.t  world frame.
# ee_orientation_base: orientation of end effector w.r.t (robot) base frame, 
#                      in the form of three angles phi, theta, psi; when axes assumes the default value these are
#                      the ZYZ euler angles.
def move_end_effector(robot_arm, ee_position_world, ee_orientation_base, axes='rzyz', wait=True, duration=1.0):
    ee_position_base = np.dot(pick_and_place_config.TRANSFORMATION_ROBOT_BASE_FRAME_WORLD_FRAME, np.append(ee_position_world, [1.]))[:-1]
    ee_orientation_base_phi, ee_orientation_base_theta, ee_orientation_base_psi = ee_orientation_base
    pose_position = np.append(ee_position_base, [0., 0., 0., 0.])
    pose_orientation = transformations.euler_matrix(
        ee_orientation_base_phi, ee_orientation_base_theta, ee_orientation_base_psi, axes=axes)
    pose = transformations.pose_quaternion_from_matrix(pose_orientation) + pose_position
    robot_arm.set_target_pose(pose=pose, wait=wait, t=duration)

def _pick_chess_piece(
        robot_arm, 
        chess_piece_type, 
        starting_cell, 
        grasp_ee_position_world_z, 
        grasp_gripper_position, 
        grasping=True, 
        wait=True, 
        duration=1.0
    ):
    
    starting_cell_position_world_x, starting_cell_position_world_y = pick_and_place_utils.get_position_world_xy(starting_cell)
    grasp_setup_ee_position_world = np.array([
        starting_cell_position_world_x, 
        starting_cell_position_world_y, 
        pick_and_place_config.GRASP_SETUP_GRIPPER_POSITION_Z
    ])
    grasp_setup_ee_position_aux = np.dot(
        pick_and_place_config.TRANSFORMATION_ROBOT_BASE_AUXILIARY_FRAME_WORLD_FRAME, 
        np.append(grasp_setup_ee_position_world, [1.])
    )
    grasp_ee_position_world = np.array([
        starting_cell_position_world_x, 
        starting_cell_position_world_y, 
        grasp_ee_position_world_z
    ])
    grasp_setup_ee_orientation_base_psi = -math.atan2(grasp_setup_ee_position_aux[1], grasp_setup_ee_position_aux[0])
    grasp_setup_ee_orientation_base = [0., math.pi, grasp_setup_ee_orientation_base_psi]
    grasp_ee_orientation_base = pick_and_place_config.DEFAULT_ORIENTATION_EULER_ZYZ

    #TODO verificare cosa cambia se l'end-effector si sposta dietro al robot
    if grasp_setup_ee_position_aux[0] * grasp_setup_ee_position_aux[1] > 0:
        grasp_setup_robot_arm_configuration = pick_and_place_config.FRONT_PICK_AND_PLACE_SETUP_ROBOT_ARM_CONFIGURATION_RIGHT_ARM
    else: 
        grasp_setup_robot_arm_configuration = pick_and_place_config.FRONT_PICK_AND_PLACE_SETUP_ROBOT_ARM_CONFIGURATION_LEFT_ARM

    robot_arm.set_joint_positions(position=grasp_setup_robot_arm_configuration, wait=wait, t=duration)
    move_end_effector(robot_arm, grasp_setup_ee_position_world, grasp_setup_ee_orientation_base, wait=wait, duration=duration)
    move_end_effector(robot_arm, grasp_ee_position_world, grasp_ee_orientation_base, wait=wait, duration=0.4)
    
    if grasping:
        chess_piece_model_name = pick_and_place_utils.get_chess_piece_model_name(
            chess_piece_type, 
            starting_cell_position_world_x, 
            starting_cell_position_world_y
        )
        chess_piece_link_name = chess_piece_model_name + '::link'
        robot_arm.gripper.command(grasp_gripper_position)
        robot_arm.gripper.grab(link_name=chess_piece_link_name)
        
    move_end_effector(robot_arm, grasp_setup_ee_position_world, grasp_ee_orientation_base, wait=wait, duration=0.4)

def _place_chess_piece(
        robot_arm, 
        chess_piece_type, 
        final_cell, 
        release_ee_position_world_z, 
        grasping=True, 
        wait=True, 
        duration=1.0
    ):

    final_cell_position_world_x, final_cell_position_world_y = pick_and_place_utils.get_position_world_xy(final_cell)
    release_setup_ee_position_world = np.array([
        final_cell_position_world_x, 
        final_cell_position_world_y, 
        pick_and_place_config.GRASP_SETUP_GRIPPER_POSITION_Z
    ])
    release_setup_ee_position_aux = np.dot(
        pick_and_place_config.TRANSFORMATION_ROBOT_BASE_AUXILIARY_FRAME_WORLD_FRAME, 
        np.append(release_setup_ee_position_world, [1.])
    )
    release_ee_position_world = np.array([
        final_cell_position_world_x, 
        final_cell_position_world_y, 
        release_ee_position_world_z
    ])
    release_setup_ee_orientation_base_psi = -math.atan2(release_setup_ee_position_aux[1], release_setup_ee_position_aux[0])
    release_setup_ee_orientation_base = [0., math.pi, release_setup_ee_orientation_base_psi]
    release_ee_orientation_base = pick_and_place_config.DEFAULT_ORIENTATION_EULER_ZYZ

    #TODO verificare cosa cambia se l'end-effector si sposta dietro al robot
    if release_setup_ee_position_aux[0] * release_setup_ee_position_aux[1] > 0:
        release_setup_robot_arm_configuration = pick_and_place_config.FRONT_PICK_AND_PLACE_SETUP_ROBOT_ARM_CONFIGURATION_RIGHT_ARM
    else: 
        release_setup_robot_arm_configuration = pick_and_place_config.FRONT_PICK_AND_PLACE_SETUP_ROBOT_ARM_CONFIGURATION_LEFT_ARM

    robot_arm.set_joint_positions(position=release_setup_robot_arm_configuration, wait=wait, t=duration)
    move_end_effector(robot_arm, release_setup_ee_position_world, release_setup_ee_orientation_base, wait=wait, duration=duration)
    move_end_effector(robot_arm, release_ee_position_world, release_ee_orientation_base, wait=wait, duration=0.4)
    
    if grasping:
        chess_piece_model_name = pick_and_place_utils.get_chess_piece_model_name(
            chess_piece_type, 
            final_cell_position_world_x, 
            final_cell_position_world_y
        )
        chess_piece_link_name = chess_piece_model_name + '::link'
        robot_arm.gripper.command(pick_and_place_config.OPEN_GRIPPER_POSITION) 
        robot_arm.gripper.release(link_name=chess_piece_link_name)
        
    move_end_effector(robot_arm, release_setup_ee_position_world, release_ee_orientation_base, wait=wait, duration=0.4)
 
@pick_and_place_utils.timeout(seconds=15, default=False)
def _move_chess_piece(
        robot_arm, 
        chess_piece_type, 
        starting_cell, 
        final_cell, 
        grasp_positions_z_dict=pick_and_place_config.DEFAULT_GRASP_POSITIONS_Z_DICT,
        grasp_gripper_positions_dict=pick_and_place_config.DEFAULT_GRASP_GRIPPER_POSITIONS_DICT, 
        grasping=True, 
        wait=True, 
        duration=1.0
    ):

    grasp_gripper_position = grasp_gripper_positions_dict[chess_piece_type]
    grasp_release_ee_position_world_z = grasp_positions_z_dict[chess_piece_type]
    _pick_chess_piece(
        robot_arm, 
        chess_piece_type, 
        starting_cell, 
        grasp_release_ee_position_world_z, 
        grasp_gripper_position, 
        grasping=grasping, 
        wait=wait, 
        duration=duration
    )
    _place_chess_piece(
        robot_arm, 
        chess_piece_type, 
        final_cell, 
        grasp_release_ee_position_world_z, 
        grasping=grasping, 
        wait=wait, 
        duration=duration
    )
    robot_arm.set_joint_positions(position=pick_and_place_config.REST_ROBOT_ARM_CONFIGURATION, wait=wait, t=duration)
    return True

def move_chess_piece(
        robot_arm, 
        chess_piece_type, 
        starting_cell, 
        final_cell, 
        grasp_positions_z_dict=pick_and_place_config.DEFAULT_GRASP_POSITIONS_Z_DICT,
        grasp_gripper_positions_dict=pick_and_place_config.DEFAULT_GRASP_GRIPPER_POSITIONS_DICT, 
        grasping=True, 
        wait=True, 
        duration=1.0
    ):

    chess_piece_moved = False
    
    while not chess_piece_moved:
        chess_piece_moved = _move_chess_piece(
            robot_arm, 
            chess_piece_type, 
            starting_cell, 
            final_cell, 
            grasp_positions_z_dict,
            grasp_gripper_positions_dict, 
            grasping, 
            wait, 
            duration
        )