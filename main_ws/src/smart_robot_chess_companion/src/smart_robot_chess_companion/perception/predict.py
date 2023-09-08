from typing import Tuple, Dict
from collections import defaultdict
import numpy as np
from ultralytics import YOLO


def predict(model_checkpoint_path: str, img: np.ndarray, device: str) -> Tuple[Dict[str, Dict[str, str]], np.ndarray]:
    
    cell_width = 63  # px
    origin = 70  # px
    ranges = np.array([[origin + cell_width * i, origin + cell_width * (i + 1) + 1] for i in range (8)])
    piece_2_name = {
        'rook': 'r',
        'knight': 'n',
        'bishop': 'b',
        'queen': 'q',
        'king': 'k',
        'pawn': 'p'
    }
        
    chess_state_dict = {}
    piece_counter = defaultdict(int)
    chess_state_matrix = np.full((8, 8), fill_value=-1, dtype=np.int8)
    
    image_h, image_w = img.shape
    
    model = YOLO(model_checkpoint_path)
    detections = model.predict(img, imgsz=image_h, conf=0.5, device=device, save_txt=False, save=False)
    
    for cur_detection in detections:
        class_names = cur_detection.names
        image_h, image_w = cur_detection.orig_shape
        bboxes = cur_detection.boxes.xywhn.numpy()
        classes = cur_detection.boxes.cls.numpy()
        for cls_idx, (x, y, w, h) in zip(classes, bboxes):
            piece, player = class_names[cls_idx].split("_")
            piece_counter[class_names[cls_idx]] += 1
            x, w = int(x * image_w), int(w * image_w)
            y, h = int(y * image_h), int(h * image_w)
            cell_col = np.where((x >= ranges[:, 0]) & (x < ranges[:, 1]))[0].item(0)
            cell_row = 7 - np.where((y >= ranges[:, 0]) & (y < ranges[:, 1]))[0].item(0)
            chess_state_matrix[cell_row, cell_col] = cls_idx
            key = f'{player}_{piece}'
            if not piece in ['queen', 'king']:
                key += f'_{piece_counter[class_names[cls_idx]]}'
            chess_state_dict[key] = {'name': piece_2_name[piece], 'col': cell_col, 'row': cell_row, 'player': player}
        
    return chess_state_dict, chess_state_matrix
