from functools import wraps
import errno
import os
import signal
import rospy
from gazebo_msgs.srv import GetModelState
from .config import *

CHESS_PIECES_MODEL_NAMES_DICT = {
    'king': [
        'king_white', 
        'king_black'
    ],
    'pawn': [
        'pawn_white_1', 'pawn_white_2', 'pawn_white_3', 'pawn_white_4', 'pawn_white_5', 'pawn_white_6', 
        'pawn_white_7', 'pawn_white_8', 
        'pawn_black_1', 'pawn_black_2', 'pawn_black_3', 'pawn_black_4', 'pawn_black_5', 'pawn_black_6', 
        'pawn_black_7', 'pawn_black_8'
    ],
    'queen': [
        'queen_white', 
        'queen_black'
    ],
    'rook': [
        'rook_white_1', 'rook_white_2',
        'rook_black_1', 'rook_black_2'
    ],
    'bishop': [
        'bishop_white_1', 'bishop_white_2', 
        'bishop_black_1', 'bishop_black_2', 
    ],
    'knight': [
        'knight_white_1', 'knight_white_2', 
        'knight_black_1', 'knight_black_2'
    ],
}

def get_position_world_xy(cell):
    if not is_valid_cell(cell):
        return (None, None)
    
    cell_size = CHESS_BOARD_SIZE[0] / 10
    cell_row = int(cell[1])
    cell_column = cell[0].lower()
    cell_row_index = 9 - cell_row
    cell_column_mapping = { column: index for index, column in enumerate(list('wyxzabcdefghijkl')) if column not in list('zi') }
    cell_column_index = cell_column_mapping[cell_column]
    cell_position_x = EXTENDED_CHESS_BOARD_CELL_WA_POSITION_WORLD_XY[0] + cell_size * cell_row_index
    cell_position_y = EXTENDED_CHESS_BOARD_CELL_WA_POSITION_WORLD_XY[1] + cell_size * cell_column_index

    return (cell_position_x, cell_position_y)

def get_chess_piece_model_name(chess_piece_type, chess_piece_position_x, chess_piece_position_y):
    model_state_service_proxy = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    model_name = None
    min_squared_distance = -1
    
    for candidate_model_name in CHESS_PIECES_MODEL_NAMES_DICT[chess_piece_type]:
        candidate_model_state = model_state_service_proxy(candidate_model_name, 'link')
        candidate_model_position = candidate_model_state.pose.position.x, candidate_model_state.pose.position.y
        squared_distance = (chess_piece_position_x - candidate_model_position[0]) ** 2 + \
                           (chess_piece_position_y - candidate_model_position[1]) ** 2
        
        if squared_distance == 0:
            return candidate_model_name
        
        if min_squared_distance == -1 or squared_distance < min_squared_distance:
            model_name = candidate_model_name
            min_squared_distance = squared_distance
    
    return model_name

def is_valid_cell(cell):
    cell_row = int(cell[1])
    cell_column = cell[0].lower()

    if cell_column not in list('wyxabcdefghjkl'):
        return False
    
    if not 1 <= cell_row <= 9:
        return False
    
    if cell_column in list('wykl') and cell_row < 5:
        return False
    
    if cell_column in list('xj') and cell_row < 4:
        return False
    
    if cell_column in list('abcdefgh') and cell_row > 8:
        return False
    
    return True

# use by adding line:
# @timeout(seconds=timer_duration_seconds, default=False)
# above the definition of a function to be timed
# n.b. said function will then return its return value if its execution doesn't take longer than timer_duration_seconds seconds, or default otherwise
def timeout(default, seconds=10, error_message=os.strerror(errno.ETIME)):
    def decorator(func):
        def _handle_timeout(signum, frame):
            raise TimeoutError(error_message)
        @wraps(func)
        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(seconds)
            try:
                result = func(*args, **kwargs)
            except TimeoutError:
                result = default
            finally:
                signal.alarm(0)
            return result
        return wrapper
    return decorator
