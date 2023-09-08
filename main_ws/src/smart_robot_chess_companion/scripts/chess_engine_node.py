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
    move_chess_piece_type = game_state.get_piece(move[0]).get_type()

    if game_state.get_piece(move[1]) == Player.EMPTY:
        move_msg = utils.dict_to_string_ros_message({
            'chess_piece_type': move_chess_piece_type, 
            'starting_cell': move_starting_square, 
            'final_cell': move_ending_square
        })

        return [move_msg]

    capture_chess_piece_type = game_state.get_piece(move[1]).get_type()

    capture_msg = ({
        'chess_piece_type': capture_chess_piece_type, 
        'starting_cell': move_ending_square, 
        'final_cell': None
    })

    move_msg = utils.dict_to_string_ros_message({
        'chess_piece_type': move_chess_piece_type, 
        'starting_cell': move_starting_square, 
        'final_cell': move_ending_square
    })

    return [capture_msg, move_msg]

def plan_user_move(game_state):
    starting_square = None
    ending_square = None

    while True:
        input_string = input("Select starting square coordinates (e.g. 'a2' for row 2 and column a): ")
        if not is_valid_input(input_string):
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
            print("Error: the chosen starting square coordinates are not valid. Try again.")
            continue

        valid_moves = game_state.get_valid_moves(starting_square)
        if not valid_moves:
            # No valid moves for the chosen starting square
            print("Error: no valid moves for the chosen piece. Try again with another piece.")
            continue

        input_string = input("Select ending square coordinates (e.g. 'a4' for row 4 and column a): ")
        while not is_valid_input(input_string):
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
            print("Error: the chosen move is not valid for the chosen piece. Try again with another move or another piece.")
            continue
        else:
            break

    return starting_square, ending_square

# board_state_dict: dictionary representation of the current board state as received from perception_node ros node
def compute_move_and_player(board_state_dict, last_board_state):
    board_state_array = np.ones((8, 8))
    last_board_state_array = np.ones((8, 8))
    board_cell_player_mapping = {}

    for chess_piece in board_state_dict:
        chess_piece_name_unicode = ord(chess_piece['name'])
        row_index = chess_piece['row_number']
        col_index = chess_piece['col_number']
        key = str(row_index) + str(col_index)
        board_cell_player_mapping[key] = chess_piece['player']
        board_state_array[row_index, col_index] = chess_piece_name_unicode
        
    for i in range(8):
        for j in range(8):
            elem_ij = last_board_state[i][j]

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

    key = str(starting_cell[0]) + str(starting_cell[1])
    player = board_cell_player_mapping[key]

    return ((starting_cell, final_cell), player)

def update_board_state(board_state_message, *args):
    input = args[0][0]
    last_timestamp = input['last_timestamp']
    board_state_message_data_dict = utils.dict_from_string_ros_message(board_state_message)
    timestamp = board_state_message_data_dict['timestamp']
    del board_state_message_data_dict['timestamp']

    if last_timestamp is None or last_timestamp != timestamp:
        input['last_timestamp'] = timestamp
        game_state = input['game_state']
        last_board_state = game_state.get_board()
        move, player = compute_move_and_player(board_state_message_data_dict, last_board_state)
        game_state.move_piece(move[0], move[1], player == "black")

def chess_engine_node():
    rospy.init_node('chess_engine_node')
    rate = rospy.Rate(2)
    move_chess_piece_command_publisher = rospy.Publisher('move_chess_piece_cmd', String, queue_size=10)
    running = True
    game_over = False
    ai = ai_engine.AIEngine()
    game_state = chess_engine.GameState()
    last_timestamp = None
    input = { 'game_state': game_state, 'last_timestamp': last_timestamp }
    board_state_subscriber = rospy.Subscriber(
        'board_state', 
        String, 
        callback=update_board_state, 
        callback_args=[input]
    )
    print("Starting the match.")
    game_state.print_board()
    
    while running:
        if not game_over:
            print("User's turn.")
            user_move = plan_user_move(game_state)

            # TODO: remove the following lines
            # game_state.move_piece(user_move[0], user_move[1], False)
            # game_state.print_board()

            # TODO: need to pass user_move to the robot system using ROS topics.
            #  The robot moves the piece for the user.
            #  Then there's the detection phase.
            #  Then, I should take the JSON file coming from the detection and update the board state.
            user_move_messages = compose_move_ros_messages(user_move, game_state)

            for msg in user_move_messages:
                move_chess_piece_command_publisher(msg)

            print("AI's turn.")
            # Due to computing and algorithmic limitations, we limit the AI to reading only three moves ahead (depth=3)
            ai_move = ai.minimax(game_state, 3, -100000, 100000, True, Player.PLAYER_2)
            
            # TODO: remove the following lines
            # game_state.move_piece(ai_move[0], ai_move[1], True)
            # print(
            #     f"AI moves from {chr(ord('a') + ai_move[0][1])}{ai_move[0][0] + 1} to {chr(ord('a') + ai_move[1][1])}{ai_move[1][0] + 1}")
            # game_state.print_board()
            
            # TODO: need to pass ai_move to the robot system using ROS topics.
            #  The robot moves the piece for the AI.
            #  Then there's NO detection.
            ai_move_messages = compose_move_ros_messages(ai_move, game_state)

            for msg in ai_move_messages:
                move_chess_piece_command_publisher(msg)

        endgame = game_state.checkmate_stalemate_checker()
        if endgame == 0:
            game_over = True  # Black wins
        elif endgame == 1:
            game_over = True  # White wins
        elif endgame == 2:
            game_over = True  # Stalemate
        
        rate.sleep() 

if __name__ == '__main__':
    try:
        chess_engine_node()
    except rospy.ROSInterruptException:
        pass