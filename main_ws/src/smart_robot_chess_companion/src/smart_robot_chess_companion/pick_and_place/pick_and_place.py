import numpy as np
from ur_control import transformations
import math
from .config import *
from .utils import *

class RobotChessCompanion():
    def __init__(
            self, 
            robot_arm, 
            robot_arm_rest_joint_configuration, 
            robot_arm_right_arm_joint_configuration,
            robot_arm_left_arm_joint_configuration, 
            grasp_configurations_dict, # dictionary containing <chess-piece-type>-(<grasp-position-world-z>,<gripper-position>) 
                                       # pairs - used for grasping chess pieces
            
            gripper_open_position=0.05,
            trajectory_time=1.0
        ):
        
        self._robot_arm = robot_arm
        self._joint_configurations = { 
            'rest': robot_arm_rest_joint_configuration,
            'right_arm': robot_arm_right_arm_joint_configuration,
            'left_arm': robot_arm_left_arm_joint_configuration
        }

        self._gripper_open_position = gripper_open_position
        self._grasp_configurations_dict = grasp_configurations_dict
        self._trajectory_time = trajectory_time if trajectory_time < TRAJECTORY_TIMEOUT_SECONDS else TRAJECTORY_TIMEOUT_SECONDS
        self.robot_arm_to_rest_configuration()
        self.open_gripper()

    def open_gripper(self):
        rospy.logdebug('Opening gripper')
        self._robot_arm.gripper.command(self._gripper_open_position)

    def close_gripper(self, chess_piece_type):
        assert chess_piece_type in self._grasp_configurations_dict
        rospy.logdebug(f'Closing gripper for chess piece "{chess_piece_type}"')
        self._robot_arm.gripper.command(self._grasp_configurations_dict[chess_piece_type][1])
    
    def robot_arm_to_rest_configuration(self):
        rospy.logdebug('Moving to rest configuration')
        self._set_joint_configuration(self._joint_configurations['rest'])

    def robot_arm_to_right_arm_configuration(self):
        rospy.logdebug('Moving to right arm configuration')
        self._set_joint_configuration(self._joint_configurations['right_arm'])

    def robot_arm_to_left_arm_configuration(self):
        rospy.logdebug('Moving to left arm configuration')
        self._set_joint_configuration(self._joint_configurations['left_arm'])

    @timeout(seconds=TRAJECTORY_TIMEOUT_SECONDS, default=False)
    def _set_end_effector_pose_timeout(self, ee_pose):
        self._robot_arm.set_target_pose(pose=ee_pose, wait=True, t=self._trajectory_time)
        return True

    def _set_end_effector_pose(self, ee_pose):
        initial_joint_configuration = self._robot_arm.joint_angles()
        
        while not self._set_end_effector_pose_timeout(ee_pose):
            warning = f'Unable to execute move to pose {ee_pose}: trying again...'
            rospy.logwarn(warning)
            self._set_joint_configuration(initial_joint_configuration)
            continue
    
    @timeout(seconds=TRAJECTORY_TIMEOUT_SECONDS, default=False)
    def _set_joint_configuration_timeout(self, joint_configuration):
        self._robot_arm.set_joint_positions(joint_configuration, wait=True, t=self._trajectory_time)
        return True

    def _set_joint_configuration(self, joint_configuration):
        initial_joint_configuration = self._robot_arm.joint_angles()
        
        while not self._set_joint_configuration_timeout(joint_configuration):
            warning = f'Unable to follow trajectory to joint configuration {joint_configuration}: trying again...'
            rospy.logwarn(warning)
            self._set_joint_configuration(initial_joint_configuration)
            continue

    # ee_position_world:   position of end effector w.r.t  world frame.
    # ee_orientation_base: orientation of end effector w.r.t (robot) base frame, 
    #                      in the form of three angles phi, theta, psi; when axes assumes the default value these are
    #                      the ZYZ euler angles.
    def robot_arm_end_effector_to_pose(self, ee_position_world, ee_orientation_base, axes='rzyz'):
        ee_position_base = np.dot(TRANSFORMATION_ROBOT_BASE_FRAME_WORLD_FRAME, np.append(ee_position_world, [1.]))[:-1]
        ee_orientation_base_phi, ee_orientation_base_theta, ee_orientation_base_psi = ee_orientation_base
        pose_position = np.append(ee_position_base, [0., 0., 0., 0.])
        pose_orientation = transformations.euler_matrix(
            ee_orientation_base_phi, ee_orientation_base_theta, ee_orientation_base_psi, axes=axes)
        pose = transformations.pose_quaternion_from_matrix(pose_orientation) + pose_position
        rospy.logdebug(f'Moving ee to ee_position_world={ee_position_world}, ee_orientation_base={ee_orientation_base}')
        self._set_end_effector_pose(pose)

    def _pick_chess_piece(
            self, 
            chess_piece_type, 
            starting_cell, 
            grasping=True, 
        ):
        
        rospy.logdebug(f'Picking up chess piece "{chess_piece_type}" in cell {starting_cell}...')
        starting_cell_position_world_x, starting_cell_position_world_y = get_position_world_xy(starting_cell)
        grasp_setup_ee_position_world = np.array([
            starting_cell_position_world_x, 
            starting_cell_position_world_y, 
            GRASP_SETUP_GRIPPER_POSITION_Z
        ])
        grasp_setup_ee_position_aux = np.dot(
            TRANSFORMATION_ROBOT_BASE_AUXILIARY_FRAME_WORLD_FRAME, 
            np.append(grasp_setup_ee_position_world, [1.])
        )
        grasp_ee_position_world = np.array([
            starting_cell_position_world_x, 
            starting_cell_position_world_y, 
            self._grasp_configurations_dict[chess_piece_type][0]
        ])
        grasp_setup_ee_orientation_base_psi = -math.atan2(grasp_setup_ee_position_aux[1], grasp_setup_ee_position_aux[0])
        grasp_setup_ee_orientation_base = [0., math.pi, grasp_setup_ee_orientation_base_psi]
        grasp_ee_orientation_base = DEFAULT_ORIENTATION_EULER_ZYZ

        if grasp_setup_ee_position_aux[0] * grasp_setup_ee_position_aux[1] > 0:
            self.robot_arm_to_right_arm_configuration()
        else: 
            self.robot_arm_to_left_arm_configuration()

        self.robot_arm_end_effector_to_pose(grasp_setup_ee_position_world, grasp_setup_ee_orientation_base)
        self.robot_arm_end_effector_to_pose(grasp_ee_position_world, grasp_ee_orientation_base)
        
        if grasping:
            chess_piece_link_name = self._get_chess_piece_link_name(
                chess_piece_type, 
                (starting_cell_position_world_x, starting_cell_position_world_y)
            )
            rospy.logdebug(f'Grabbing chess piece "{chess_piece_type}"...')
            self.close_gripper(chess_piece_type)
            self._robot_arm.gripper.grab(link_name=chess_piece_link_name)
            
        self.robot_arm_end_effector_to_pose(grasp_setup_ee_position_world, grasp_ee_orientation_base)

    def _place_chess_piece(
            self, 
            chess_piece_type, 
            final_cell, 
            grasping=True, 
        ):

        rospy.logdebug(f'Placing chess piece "{chess_piece_type}" in cell {final_cell}...')
        final_cell_position_world_x, final_cell_position_world_y = get_position_world_xy(final_cell)

        release_setup_ee_position_world = np.array([
            final_cell_position_world_x, 
            final_cell_position_world_y, 
            GRASP_SETUP_GRIPPER_POSITION_Z
        ])
        release_setup_ee_position_aux = np.dot(
            TRANSFORMATION_ROBOT_BASE_AUXILIARY_FRAME_WORLD_FRAME, 
            np.append(release_setup_ee_position_world, [1.])
        )
        release_ee_position_world = np.array([
            final_cell_position_world_x, 
            final_cell_position_world_y, 
            self._grasp_configurations_dict[chess_piece_type][0]
        ])
        release_setup_ee_orientation_base_psi = -math.atan2(release_setup_ee_position_aux[1], release_setup_ee_position_aux[0])
        release_setup_ee_orientation_base = [0., math.pi, release_setup_ee_orientation_base_psi]
        release_ee_orientation_base = DEFAULT_ORIENTATION_EULER_ZYZ

        if release_setup_ee_position_aux[0] * release_setup_ee_position_aux[1] > 0:
            self.robot_arm_to_right_arm_configuration()
        else: 
            self.robot_arm_to_left_arm_configuration()

        self.robot_arm_end_effector_to_pose(release_setup_ee_position_world, release_setup_ee_orientation_base)
        self.robot_arm_end_effector_to_pose(release_ee_position_world, release_ee_orientation_base)
        
        if grasping:
            chess_piece_link_name = self._get_chess_piece_link_name(
                chess_piece_type, 
                (final_cell_position_world_x, final_cell_position_world_y)
            )
            rospy.logdebug(f'Releasing chess piece "{chess_piece_type}"...')
            self.open_gripper()
            self._robot_arm.gripper.release(link_name=chess_piece_link_name)
            
        self.robot_arm_end_effector_to_pose(release_setup_ee_position_world, release_ee_orientation_base)
 
    def move_chess_piece(
            self, 
            chess_piece_type, 
            starting_cell, 
            final_cell, 
            grasping=True, 
        ):

        self._pick_chess_piece(chess_piece_type, starting_cell, grasping)
        self._place_chess_piece(chess_piece_type, final_cell, grasping)
        self.robot_arm_to_rest_configuration()

    def _get_chess_piece_link_name(self, chess_piece_type, cell_position_world_xy):
        chess_piece_model_name = get_chess_piece_model_name(
                chess_piece_type, 
                cell_position_world_xy[0], 
                cell_position_world_xy[1]
        )
        chess_piece_link_name = chess_piece_model_name + '::link'
        return chess_piece_link_name











        



    
