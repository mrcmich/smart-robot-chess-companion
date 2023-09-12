import math
import numpy as np

# timings
TRAJECTORY_TIMEOUT_SECONDS = 5

# position and size of relevant models
ROBOT_BASE_POSITION_WORLD_X = -0.315
TABLE_HEIGHT = 0.775
CHESS_BOARD_SIZE = (0.440, 0.440, 0.03)
EXTENDED_CHESS_BOARD_CELL_WA_POSITION_WORLD_XY = np.array([-4.5, -7.5]) * CHESS_BOARD_SIZE[0] / 10

# robot arm joint configurations and gripper positions
REST_ROBOT_ARM_CONFIGURATION = [-1.57, -3.3, 2., -1.1, -1.57, 0.]
RIGHT_ARM_ROBOT_ARM_CONFIGURATION = [1.57, -1.57, -1.57, -1.57, 1.57, 0.]
LEFT_ARM_ROBOT_ARM_CONFIGURATION = [-1.57, -1.57, 1.57, -1.57, -1.57, 0.]
OPEN_GRIPPER_POSITION = 0.035

# grasping pose configuration
GRIPPER_POSITION_Z_MARGIN = 0.05
Z_DISTANCE_ROBOT_WRIST3_GRIPPER_FINGER_FRAMES = 0.107
GRIPPER_POSITION_Z_MARGIN = 0.05
MIN_GRIPPER_POSITION_Z = TABLE_HEIGHT + CHESS_BOARD_SIZE[-1] + Z_DISTANCE_ROBOT_WRIST3_GRIPPER_FINGER_FRAMES + \
                         GRIPPER_POSITION_Z_MARGIN
GRASP_SETUP_GRIPPER_POSITION_Z = MIN_GRIPPER_POSITION_Z + 0.1
DEFAULT_GRASP_CONFIGURATIONS_DICT = {
    'king':   (MIN_GRIPPER_POSITION_Z + 0.029, 0.0223),
    'pawn':   (MIN_GRIPPER_POSITION_Z + 0.019, 0.0130),
    'queen':  (MIN_GRIPPER_POSITION_Z + 0.029, 0.0223),
    'rook':   (MIN_GRIPPER_POSITION_Z + 0.020, 0.0200),
    'bishop': (MIN_GRIPPER_POSITION_Z + 0.020, 0.0180),
    'knight': (MIN_GRIPPER_POSITION_Z + 0.025, 0.0130)
}

# euler ZYZ orientations
DEFAULT_ORIENTATION_EULER_ZYZ = [0., math.pi, -math.pi]

# static homogeneous transformations
TRANSFORMATION_ROBOT_BASE_FRAME_WORLD_FRAME = np.array([
    [ 0., 1., 0.,  0.],
    [-1., 0., 0.,  ROBOT_BASE_POSITION_WORLD_X],
    [ 0., 0., 1., -TABLE_HEIGHT],
    [ 0., 0., 0.,  1.]
])
TRANSFORMATION_ROBOT_BASE_AUXILIARY_FRAME_WORLD_FRAME = np.array([
    [0., -1., 0.,  0.],
    [1.,  0., 0., -ROBOT_BASE_POSITION_WORLD_X],
    [0.,  0., 1., -TABLE_HEIGHT],
    [0.,  0., 0.,  1.]
])

# mappings
MAPPING_N_CAPTURED_CHESS_PIECES_CELL = { n_captured_chess_pieces: cell for n_captured_chess_pieces, cell in enumerate([
    'x9', 'x8', 'x7', 'x6', 'x5', 'x4',
    'y9', 'y8', 'y7', 'y6', 'y5',
    'w9', 'w8', 'w7', 'w6', 'w5',
    'j9', 'j8', 'j7', 'j6', 'j5', 'j4',
    'k9', 'k8', 'k7', 'k6', 'k5',
    'l9', 'l8', 'l7', 'l6', 'l5'
]) }
