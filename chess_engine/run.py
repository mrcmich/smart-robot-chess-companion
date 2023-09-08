"""
Assume that human player controls white pieces (Player.PLAYER_1) and AI player controls black pieces (Player.PLAYER_2).
White always makes the first move.
"""

import chess_engine
import re

import ai_engine
from player import Player


def is_valid_input(input_string):
    # Define the regular expression pattern (letter a-h for the columns, number 1-8 for the rows)
    pattern = r'^[a-h][1-8]$'
    # Check if the input matches the pattern
    match = re.match(pattern, input_string)
    # If there's a match, return True; otherwise, return False
    return bool(match)


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


def run():
    print("Starting the match.")
    running = True
    game_over = False

    ai = ai_engine.AIEngine()
    game_state = chess_engine.GameState()
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

        endgame = game_state.checkmate_stalemate_checker()
        if endgame == 0:
            game_over = True  # Black wins
        elif endgame == 1:
            game_over = True  # White wins
        elif endgame == 2:
            game_over = True  # Stalemate


if __name__ == "__main__":
    run()
