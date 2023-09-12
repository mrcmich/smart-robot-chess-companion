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

MAPPING_CHESS_PIECE_TYPE_NUMERIC_ID = {chess_piece: index+2 for index, chess_piece in enumerate(Player.PIECES)}

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
    # rospy.logdebug(f'move_chess_piece_type -> {type(game_state.get_piece(*move[0]))}')
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
        input_string = input("\nSelect starting square coordinates (e.g. 'a2' for row 2 and column a): ")
        if not is_valid_input(input_string):
            # rospy.logdebug("Error: wrong input format. Try again.")
            print("Error: wrong input format. Try again.")
            continue
        row = int(input_string[1])
        col = int(ord(input_string[0]) - ord('a') + 1)  # to map the character [a-h] in [1-8]

        # Now we need to map numbers from (1, 8) to (0, 7)
        row -= 1
        col -= 1

        starting_square = (row, col)

        if not game_state.is_valid_piece(row, col) or not game_state.get_piece(row, col).is_player(Player.PLAYER_1):
            # Selected starting square is not valid
            # rospy.logdebug("Error: the chosen starting square coordinates are not valid. Try again.")
            print("Error: the chosen starting square coordinates are not valid. Try again.")
            continue

        valid_moves = game_state.get_valid_moves(starting_square)
        if not valid_moves:
            # No valid moves for the chosen starting square
            # rospy.logdebug("Error: no valid moves for the chosen piece. Try again with another piece.")
            print("Error: no valid moves for the chosen piece. Try again with another piece.")
            continue

        input_string = input("Select ending square coordinates (e.g. 'a4' for row 4 and column a): ")
        while not is_valid_input(input_string):
            # rospy.logdebug("Error: wrong input format. Try again.\n")
            print("Error: wrong input format. Try again.\n")
            input_string = input("Select ending square coordinates (e.g. 'a4' for row 4 and column a): ")
        row = int(input_string[1])
        col = int(ord(input_string[0]) - ord('a') + 1)  # to map the character [a-h] in [1-8]

        # Now we need to map numbers from (1, 8) to (0, 7)
        row -= 1
        col -= 1

        ending_square = (row, col)

        # Check that the selected ending square is a valid move
        if ending_square not in valid_moves:
            # rospy.logdebug("Error: the chosen move is not valid for the chosen piece. Try again with another move or another piece.")
            print("Error: the chosen move is not valid for the chosen piece. Try again with another move or another piece.")
            continue
        else:
            break

    return starting_square, ending_square

def compute_move(game_state, board_state_update_array):
    last_board_state_array = np.ones((8, 8))
    
    for i in range(8):
        for j in range(8):
            elem_ij = game_state.get_piece(i, j)

            if isinstance(elem_ij, Piece):
                elem_ij_type = f'{elem_ij.get_player()}_{elem_ij.get_name()}'
                elem_ij_numeric_id = MAPPING_CHESS_PIECE_TYPE_NUMERIC_ID[elem_ij_type]
                last_board_state_array[i][j] = elem_ij_numeric_id

    board_state_difference = board_state_update_array - last_board_state_array
    
    #dbg
    assert (board_state_difference != 0).any()
    
    nz = board_state_difference.nonzero()
    i1, j1 = nz[0][0], nz[1][0]
    i2, j2 = nz[0][1], nz[1][1]

    if board_state_update_array[i1, j1] == 1:
        starting_cell = i1, j1
        final_cell = i2, j2
    else:
        starting_cell = i2, j2
        final_cell = i1, j1   
    
    # rospy.logdebug(f'move: {starting_cell}, {final_cell}')
    return starting_cell, final_cell
        
def extract_board_state_update_array_from_ros_message(message, *args):
    input = args[0][0]
    output = args[0][1]
    last_timestamp = input['last_timestamp']
    board_state_message_data_dict = utils.dict_from_string_ros_message(message)
    timestamp = board_state_message_data_dict['timestamp']
    del board_state_message_data_dict['timestamp']
    
    if last_timestamp == timestamp:
        return  
       
    input['last_timestamp'] = timestamp
    board_state_array = np.ones((8, 8))
        
    for chess_piece in board_state_message_data_dict.values():
        chess_piece_type = f"{chess_piece['player']}_{chess_piece['name']}"
        row_index = chess_piece['row_number']
        col_index = chess_piece['col_number']
        board_state_array[row_index, col_index] = MAPPING_CHESS_PIECE_TYPE_NUMERIC_ID[chess_piece_type]
        
    # rospy.logdebug(f'Received state of pieces: {list(board_state_message_data_dict.keys())}')
    output.append(board_state_array)

def chess_engine_node():
    # rospy.init_node('chess_engine_node', anonymous=True, log_level=rospy.DEBUG)
    rospy.init_node('chess_engine_node', anonymous=True)
    rate = rospy.Rate(2)
    move_chess_piece_command_publisher = rospy.Publisher('ur3e_puppeteer_node_cmd', String, queue_size=10)
    is_user_turn = True
    game_over = False
    ai = ai_engine.AIEngine()
    game_state = chess_engine.GameState()
    input = { 'game_state': game_state, 'last_timestamp': None }
    output = []
    board_state_subscriber = rospy.Subscriber(
        'board_state', 
        String, 
        callback=extract_board_state_update_array_from_ros_message, 
        callback_args=[input, output],
        queue_size=10
    )
    is_first_turn = True
    rate.sleep()

    # rospy.logdebug("Starting the match.")
    print("Starting the match.")

    try:
        while not rospy.is_shutdown():
            if not game_over:
                if is_first_turn is False and len(output) == 0:
                    rate.sleep()
                    continue
            
                if is_user_turn:
                    if is_first_turn:
                        is_first_turn = False
                    else:
                        board_state_update_array = output.pop(0)
                        next_move = compute_move(game_state, board_state_update_array)
                        game_state.move_piece(starting_square=next_move[0], ending_square=next_move[1], is_ai=False)
                        
                    # rospy.logdebug("User's turn.")
                    print("\nUser's turn.")
                    game_state.print_board()
                    user_move = plan_user_move(game_state)
                    user_move_message = compose_move_ros_messages(user_move, game_state)
                    move_chess_piece_command_publisher.publish(user_move_message)
                    # rospy.logdebug(f'User moves from {to_string_representation(user_move[0])} to {to_string_representation(user_move[1])}')
                    print(f"\nUser moves from '{to_string_representation(user_move[0])}' to '{to_string_representation(user_move[1])}'")
                    is_user_turn = False
                else:
                    board_state_update_array = output.pop(0)
                    next_move = compute_move(game_state, board_state_update_array)
                    game_state.move_piece(starting_square=next_move[0], ending_square=next_move[1], is_ai=True)
                    # rospy.logdebug("AI's turn.")
                    print("\nAI's turn.")
                    game_state.print_board()
                    # Due to computing and algorithmic limitations, we limit the AI to reading only three moves ahead (depth=3)
                    ai_move = ai.minimax(game_state, 3, -100000, 100000, True, Player.PLAYER_2)
                    ai_move_message = compose_move_ros_messages(ai_move, game_state)
                    move_chess_piece_command_publisher.publish(ai_move_message)
                    # rospy.logdebug(f"AI moves from {to_string_representation(ai_move[0])} to {to_string_representation(ai_move[1])}.")
                    print(f"\nAI moves from '{to_string_representation(ai_move[0])}' to '{to_string_representation(ai_move[1])}'.")
                    is_user_turn = True

                endgame = game_state.checkmate_stalemate_checker()
                if endgame == 0:
                    game_over = True
                    # rospy.logdebug("Black wins.")
                    print("Black wins.")
                elif endgame == 1:
                    game_over = True
                    # rospy.logdebug("White wins.")
                    print("White wins.")
                elif endgame == 2:
                    game_over = True
                    # rospy.logdebug("Stalemate.")
                    print("Stalemate.")
            
                rate.sleep()
    
    except KeyboardInterrupt:
        print("Terminated. Good bye.")
        rospy.signal_shutdown("Terminated. Good bye.")

if __name__ == '__main__':
    try:
        chess_engine_node()
    except rospy.ROSInterruptException:
        pass
