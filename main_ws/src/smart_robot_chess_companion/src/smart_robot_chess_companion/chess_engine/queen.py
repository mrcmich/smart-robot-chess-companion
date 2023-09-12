from .rook import Rook
from .bishop import Bishop


class Queen(Rook, Bishop):
    def __init__(self, name, row_number, col_number, player):
        super().__init__(name, row_number, col_number, player)
        self._type = 'queen'
    
    def get_type(self):
        return self._type

    def get_valid_piece_moves(self, game_state):
        return (Rook.get_valid_piece_moves(Rook(self.get_name(), self.get_row_number(), self.get_col_number(), self.get_player()),
                                           game_state) +
                Bishop.get_valid_piece_moves(Bishop(self.get_name(), self.get_row_number(), self.get_col_number(), self.get_player()),
                                             game_state))
