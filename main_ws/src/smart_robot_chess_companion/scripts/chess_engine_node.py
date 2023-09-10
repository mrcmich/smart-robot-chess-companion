#! /usr/bin/env python

"""
Assume that human player controls white pieces (Player.PLAYER_1) and AI player controls black pieces (Player.PLAYER_2).
White always makes the first move.
"""

import smart_robot_chess_companion.chess_engine.chess_engine as chess_engine
import smart_robot_chess_companion.utils as utils
import smart_robot_chess_companion.chess_engine.ai_engine as ai_engine
from smart_robot_chess_companion.chess_engine.piece import Piece
from smart_robot_chess_companion.chess_engine.player import Player
import re
import numpy as np
import rospy
from std_msgs.msg import String


def is_valid_input(input_string):
    # Define the regular expression pattern (letter a-h for the columns, number 1-8 for the rows)
    pattern = r'^[a-h][1-8]$'
    # Check if the input matches the pattern
    match = re.match(pattern, input_string)
    # If there's a match, return True; otherwise, return False
    return bool(match)

def to_string_representation(chess_board_square_coordinates):
    chess_board_square = f"{ chr(ord('a') + chess_board_square_coordinates[1]) }{ chess_board_square_coordinates[0] + 1 }"
    return chess_board_square

def compose_move_ros_messages(move, game_state):
    move_starting_square = to_string_representation(move[0])
    move_ending_square = to_string_representation(move[1])
    rospy.logdebug(f'move_chess_piece_type -> {type(game_state.get_piece(*move[0]))}')
    move_chess_piece_type = game_state.get_piece(*move[0]).get_type()

    move_dict = {
        'chess_piece_type': move_chess_piece_type, 
        'starting_cell': move_starting_square, 
        'final_cell': move_ending_square
    }

    capture_dict = None
    if game_state.get_piece(*move[1]) != Player.EMPTY:
        capture_chess_piece_type = game_state.get_piece(*move[1]).get_type()
        capture_dict = {
            'chess_piece_type': capture_chess_piece_type, 
            'starting_cell': move_ending_square, 
            'final_cell': None
        }

    return utils.dict_to_string_ros_message({
        'move': move_dict,
        'capture': capture_dict
    })

def plan_user_move(game_state):
    starting_square = None
    ending_square = None

    while True:
        input_string = input("Select starting square coordinates (e.g. 'a2' for row 2 and column a): ")
        if not is_valid_input(input_string):
            rospy.logdebug("Error: wrong input format. Try again.")
            continue
        row = int(input_string[1])
        col = int(ord(input_string[0]) - ord('a') + 1)  # to map the character [a-h] in [1-8]

        # Now we need to map numbers from (1, 8) to (0, 7)
        row -= 1
        col -= 1

        starting_square = (row, col)

        if not game_state.is_valid_piece(row, col) or not game_state.get_piece(row, col).is_player(Player.PLAYER_1):
            # Selected starting square is not valid
            rospy.logdebug("Error: the chosen starting square coordinates are not valid. Try again.")
            continue

        valid_moves = game_state.get_valid_moves(starting_square)
        if not valid_moves:
            # No valid moves for the chosen starting square
            rospy.logdebug("Error: no valid moves for the chosen piece. Try again with another piece.")
            continue

        input_string = input("Select ending square coordinates (e.g. 'a4' for row 4 and column a): ")
        while not is_valid_input(input_string):
            rospy.logdebug("Error: wrong input format. Try again.\n")
            input_string = input("Select ending square coordinates (e.g. 'a4' for row 4 and column a): ")
        row = int(input_string[1])
        col = int(ord(input_string[0]) - ord('a') + 1)  # to map the character [a-h] in [1-8]

        # Now we need to map numbers from (1, 8) to (0, 7)
        row -= 1
        col -= 1

        ending_square = (row, col)

        # Check that the selected ending square is a valid move
        if ending_square not in valid_moves:
            rospy.logdebug("Error: the chosen move is not valid for the chosen piece. Try again with another move or another piece.")
            continue
        else:
            break

    return starting_square, ending_square

def update_board_state(board_state_message, *args):
    input = args[0][0]
    last_timestamp = input['last_timestamp']
    board_state_message_data_dict = utils.dict_from_string_ros_message(board_state_message)
    timestamp = board_state_message_data_dict['timestamp']
    del board_state_message_data_dict['timestamp']
    
    # uso di board_state_message_data_dict fittizio per debugging dopo mossa d2 -> d4
    '''
    board_state_message_data_dict = {
        'white_pawn_1': {'name': 'p', 'col_number': 0, 'row_number': 1, 'player': 'white'},
        'white_pawn_2': {'name': 'p', 'col_number': 1, 'row_number': 1, 'player': 'white'},
        'white_pawn_3': {'name': 'p', 'col_number': 2, 'row_number': 1, 'player': 'white'},
        'white_pawn_4': {'name': 'p', 'col_number': 3, 'row_number': 3, 'player': 'white'},
        'white_pawn_5': {'name': 'p', 'col_number': 4, 'row_number': 1, 'player': 'white'},
        'white_pawn_6': {'name': 'p', 'col_number': 5, 'row_number': 1, 'player': 'white'},
        'white_pawn_7': {'name': 'p', 'col_number': 6, 'row_number': 1, 'player': 'white'},
        'white_pawn_8': {'name': 'p', 'col_number': 7, 'row_number': 1, 'player': 'white'},
        'white_rook_1':   {'name': 'r', 'col_number': 0, 'row_number': 0, 'player': 'white'},
        'white_knight_1': {'name': 'n', 'col_number': 1, 'row_number': 0, 'player': 'white'},
        'white_bishop_1': {'name': 'b', 'col_number': 2, 'row_number': 0, 'player': 'white'},
        'white_queen':    {'name': 'q', 'col_number': 3, 'row_number': 0, 'player': 'white'},
        'white_king':     {'name': 'k', 'col_number': 4, 'row_number': 0, 'player': 'white'},
        'white_bishop_2': {'name': 'b', 'col_number': 5, 'row_number': 0, 'player': 'white'},
        'white_knight_2': {'name': 'n', 'col_number': 6, 'row_number': 0, 'player': 'white'},
        'white_rook_2':   {'name': 'r', 'col_number': 7, 'row_number': 0, 'player': 'white'},
        'black_pawn_1': {'name': 'p', 'col_number': 0, 'row_number': 6, 'player': 'black'},
        'black_pawn_2': {'name': 'p', 'col_number': 1, 'row_number': 6, 'player': 'black'},
        'black_pawn_3': {'name': 'p', 'col_number': 2, 'row_number': 6, 'player': 'black'},
        'black_pawn_4': {'name': 'p', 'col_number': 3, 'row_number': 6, 'player': 'black'},
        'black_pawn_5': {'name': 'p', 'col_number': 4, 'row_number': 6, 'player': 'black'},
        'black_pawn_6': {'name': 'p', 'col_number': 5, 'row_number': 6, 'player': 'black'},
        'black_pawn_7': {'name': 'p', 'col_number': 6, 'row_number': 6, 'player': 'black'},
        'black_pawn_8': {'name': 'p', 'col_number': 7, 'row_number': 6, 'player': 'black'},
        'black_rook_1':   {'name': 'r', 'col_number': 0, 'row_number': 7, 'player': 'black'},
        'black_knight_1': {'name': 'n', 'col_number': 1, 'row_number': 7, 'player': 'black'},
        'black_bishop_1': {'name': 'b', 'col_number': 2, 'row_number': 7, 'player': 'black'},
        'black_queen':    {'name': 'q', 'col_number': 3, 'row_number': 7, 'player': 'black'},
        'black_king':     {'name': 'k', 'col_number': 4, 'row_number': 7, 'player': 'black'},
        'black_bishop_2': {'name': 'b', 'col_number': 5, 'row_number': 7, 'player': 'black'},
        'black_knight_2': {'name': 'n', 'col_number': 6, 'row_number': 7, 'player': 'black'},
        'black_rook_2':   {'name': 'r', 'col_number': 7, 'row_number': 7, 'player': 'black'},
    }
    '''

    if not input['is_user_turn'] and last_timestamp != timestamp:
        input['last_timestamp'] = timestamp
        game_state = input['game_state']

        board_state_array = np.ones((8, 8))
        last_board_state_array = np.ones((8, 8))
        board_cell_player_mapping = {}

        for chess_piece in board_state_message_data_dict.values():
            chess_piece_name_unicode = ord(chess_piece['name'])
            row_index = chess_piece['row_number']
            col_index = chess_piece['col_number']
            key = str(row_index) + str(col_index)
            board_cell_player_mapping[key] = chess_piece['player']
            board_state_array[row_index, col_index] = chess_piece_name_unicode
        
        rospy.logdebug(f'Received state of pieces: {list(board_state_message_data_dict.keys())}')
            
        for i in range(8):
            for j in range(8):
                elem_ij = game_state.get_piece(i, j)

                if isinstance(elem_ij, Piece):
                    last_board_state_array[i][j] = ord(elem_ij.get_name())

        board_state_difference = board_state_array - last_board_state_array
        nz = board_state_difference.nonzero()
        i1, j1 = nz[0][0], nz[1][0]
        i2, j2 = nz[0][1], nz[1][1]

        if board_state_array[i1, j1] == 1:
            starting_cell = i1, j1
            final_cell = i2, j2
        else:
            starting_cell = i2, j2
            final_cell = i1, j1   

        key = str(final_cell[0]) + str(final_cell[1])
        player = board_cell_player_mapping[key]
        rospy.logdebug(f'move: {starting_cell}, {final_cell}; player: {player}')
        game_state.move_piece(starting_cell, final_cell, player == "black")

def chess_engine_node():
    rospy.init_node('chess_engine_node', anonymous=True, log_level=rospy.DEBUG)
    rate = rospy.Rate(2)
    move_chess_piece_command_publisher = rospy.Publisher('ur3e_puppeteer_node_cmd', String, queue_size=10)
    is_user_turn = True
    game_over = False
    ai = ai_engine.AIEngine()
    game_state = chess_engine.GameState()
    last_timestamp = None
    input = { 'game_state': game_state, 'last_timestamp': last_timestamp, 'is_user_turn': is_user_turn }
    board_state_subscriber = rospy.Subscriber(
        'board_state', 
        String, 
        callback=update_board_state, 
        callback_args=[input]
    )
    rospy.logdebug("Starting the match.")
    game_state.print_board()
    
    while not rospy.is_shutdown():
        if not game_over:
            if is_user_turn:
                rospy.logdebug("User's turn.")
                user_move = plan_user_move(game_state)
                user_move_message = compose_move_ros_messages(user_move, game_state)
                move_chess_piece_command_publisher.publish(user_move_message)
                is_user_turn = False
                input['is_user_turn'] = False
            
            elif input['last_timestamp'] is not None and last_timestamp != input['last_timestamp']:
                rospy.logdebug("AI's turn.")
                game_state.print_board()
                # Due to computing and algorithmic limitations, we limit the AI to reading only three moves ahead (depth=3)
                ai_move = ai.minimax(game_state, 3, -100000, 100000, True, Player.PLAYER_2)
                ai_move_message = compose_move_ros_messages(ai_move, game_state)
                move_chess_piece_command_publisher.publish(ai_move_message)
                game_state.move_piece(starting_square=ai_move[0], ending_square=ai_move[1], is_ai=True)
                rospy.logdebug(f"AI moves from {to_string_representation(ai_move[0])} to {to_string_representation(ai_move[1])}.")
                game_state.print_board()
                is_user_turn = True
                input['is_user_turn'] = is_user_turn
                last_timestamp = input['last_timestamp']

        endgame = game_state.checkmate_stalemate_checker()
        if endgame == 0:
            game_over = True
            rospy.logdebug("Black wins.")
        elif endgame == 1:
            game_over = True
            rospy.logdebug("White wins.")
        elif endgame == 2:
            game_over = True
            rospy.logdebug("Stalemate.")
        
        rate.sleep() 

if __name__ == '__main__':
    try:
        chess_engine_node()
    except rospy.ROSInterruptException:
        pass
