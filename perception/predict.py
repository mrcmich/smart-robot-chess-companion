from pathlib import Path
from argparse import Namespace
import json
from collections import defaultdict
import cv2
import numpy as np
import torch
from ultralytics import YOLO

from utils import parse_arguments


def predict(args: Namespace) -> None:

    # model = YOLO(args.checkpoint_path)
    model = YOLO("/Users/francescobaraldi/Downloads/best.pt")
    input_dir = Path("/Users/francescobaraldi/Desktop/untitled folder")
    output_dir = Path("/Users/francescobaraldi/Desktop/chess_dataset/test/pred_images")
    output_dir.mkdir(exist_ok=True, parents=True)
    
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
    
    for image_filename in input_dir.glob("*"):
        if image_filename.stem.startswith("."):
            continue
        chess_state = {}
        piece_counter = defaultdict(int)
        chess_state_matrix = np.full((8, 8), fill_value=-1, dtype=np.int8)
        results = model.predict(str(image_filename), imgsz=args.image_size_h, conf=0.5, device=args.device, save_txt=False, save=False)
        for cur_result in results:
            img = cur_result.orig_img
            class_names = cur_result.names
            image_h, image_w = cur_result.orig_shape
            bboxes = cur_result.boxes.xywhn.numpy()
            classes = cur_result.boxes.cls.numpy()
            for cls_idx, (x, y, w, h) in zip(classes, bboxes):
                piece, player = class_names[cls_idx].split("_")
                piece_counter[class_names[cls_idx]] += 1
                x, w = int(x * image_w), int(w * image_w)
                y, h = int(y * image_h), int(h * image_w)
                color = (0, 0, 0) if player == 'black' else (255, 255, 255)
                cv2.rectangle(img, (x - w // 2, y - h // 2), ((x + (w - w // 2), y + (h - h // 2))), color=color, thickness=2)
                cv2.putText(img, piece, org=(x - w // 2, y - h // 2 - 10), fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale=1, color=color, thickness=2)
                cv2.imwrite(str(output_dir / image_filename.name), img)
                cell_col = np.where((x >= ranges[:, 0]) & (x < ranges[:, 1]))[0].item(0)
                cell_row = 7 - np.where((y >= ranges[:, 0]) & (y < ranges[:, 1]))[0].item(0)
                chess_state_matrix[cell_row, cell_col] = cls_idx
                key = f'{player}_{piece}'
                if not piece in ['queen', 'king']:
                    key += f'_{piece_counter[class_names[cls_idx]]}'
                chess_state[key] = {'name': piece_2_name[piece], 'col': cell_col, 'row': cell_row, 'player': player}
            chess_state_json_str = json.dumps(chess_state)
            # send to paglia the json str if the state has changed
            

if __name__ == "__main__":
    
    args = parse_arguments()
    assert args.image_size_h == args.image_size_w, "ERROR: during training image_size_h must be equal to image_size_w"
    predict(args=args)
