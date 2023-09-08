from pieces.rook import Rook
from pieces.bishop import Bishop


class Queen(Rook, Bishop):
    def get_valid_piece_moves(self, game_state):
        return (Rook.get_valid_piece_moves(Rook(self.get_name(), self.get_row_number(), self.get_col_number(), self.get_player()),
                                           game_state) +
                Bishop.get_valid_piece_moves(Bishop(self.get_name(), self.get_row_number(), self.get_col_number(), self.get_player()),
                                             game_state))
