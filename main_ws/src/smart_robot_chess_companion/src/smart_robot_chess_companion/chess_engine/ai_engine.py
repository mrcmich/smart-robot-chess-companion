from player import Player


class AIEngine:
    """
    Call minimax with alpha beta pruning, evaluate board and get the value of each piece.
    """

    def minimax(self, game_state, depth, alpha, beta, maximizing_player, player_color):
        # This method returns 0 if white lost, 1 if black lost, 2 if stalemate, 3 if not game over
        csc = game_state.checkmate_stalemate_checker()
        if maximizing_player:
            if csc == 0:
                return 5000000
            elif csc == 1:
                return -5000000
            elif csc == 2:
                return 100
        elif not maximizing_player:
            if csc == 1:
                return 5000000
            elif csc == 0:
                return -5000000
            elif csc == 2:
                return 100

        if depth <= 0 or csc != 3:
            return self.evaluate_board(game_state, Player.PLAYER_1)

        if maximizing_player:
            # Maximize black player

            max_evaluation = -10000000
            all_possible_moves = game_state.get_all_legal_moves("black")
            for move_pair in all_possible_moves:
                game_state.move_piece(move_pair[0], move_pair[1], True)

                evaluation = self.minimax(game_state, depth - 1, alpha, beta, False, "white")
                game_state.undo_move()

                if max_evaluation < evaluation:
                    max_evaluation = evaluation
                    best_possible_move = move_pair
                alpha = max(alpha, evaluation)
                if beta <= alpha:
                    break
            if depth == 3:
                return best_possible_move
            else:
                return max_evaluation
        else:
            # Minimize white player

            min_evaluation = 10000000
            all_possible_moves = game_state.get_all_legal_moves("white")
            for move_pair in all_possible_moves:
                game_state.move_piece(move_pair[0], move_pair[1], True)
                evaluation = self.minimax(game_state, depth - 1, alpha, beta, True, "black")
                game_state.undo_move()

                if min_evaluation > evaluation:
                    min_evaluation = evaluation
                    best_possible_move = move_pair
                beta = min(beta, evaluation)
                if beta <= alpha:
                    break
            if depth == 3:
                return best_possible_move
            else:
                return min_evaluation

    def evaluate_board(self, game_state, player):
        evaluation_score = 0
        for row in range(0, 8):
            for col in range(0, 8):
                if game_state.is_valid_piece(row, col):
                    evaluated_piece = game_state.get_piece(row, col)
                    evaluation_score += self.get_piece_value(evaluated_piece, player)
        return evaluation_score

    @staticmethod
    def get_piece_value(piece, player):
        if player is Player.PLAYER_1:
            if piece.is_player("black"):
                if piece.get_name() == "k":
                    return 1000
                elif piece.get_name() == "q":
                    return 100
                elif piece.get_name() == "r":
                    return 50
                elif piece.get_name() == "b":
                    return 30
                elif piece.get_name() == "n":
                    return 30
                elif piece.get_name() == "p":
                    return 10
            else:
                if piece.get_name() == "k":
                    return -1000
                elif piece.get_name() == "q":
                    return -100
                elif piece.get_name() == "r":
                    return -50
                elif piece.get_name() == "b":
                    return -30
                elif piece.get_name() == "n":
                    return -30
                elif piece.get_name() == "p":
                    return -10
        else:
            if piece.is_player("white"):
                if piece.get_name() == "k":
                    return 1000
                elif piece.get_name() == "q":
                    return 100
                elif piece.get_name() == "r":
                    return 50
                elif piece.get_name() == "b":
                    return 30
                elif piece.get_name() == "n":
                    return 30
                elif piece.get_name() == "p":
                    return 10
            else:
                if piece.get_name() == "k":
                    return -1000
                elif piece.get_name() == "q":
                    return -100
                elif piece.get_name() == "r":
                    return -50
                elif piece.get_name() == "b":
                    return -30
                elif piece.get_name() == "n":
                    return -30
                elif piece.get_name() == "p":
                    return -10
