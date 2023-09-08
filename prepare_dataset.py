from pathlib import Path
from argparse import ArgumentParser, Namespace
from itertools import product
import random
import cv2
import numpy as np


def generate_world_files(input_filename: Path, output_dir: Path, num_files: int) -> None:
    
    dx = dy = 75
    
    class_names = {
        "pawn_white": 0,
        "rook_white": 1,
        "knight_white": 2,
        "bishop_white": 3,
        "queen_white": 4,
        "king_white": 5,
        "pawn_black": 6,
        "rook_black": 7,
        "knight_black": 8,
        "bishop_black": 9,
        "queen_black": 10,
        "king_black": 11,
    }
    
    pieces_names = [
        "pawn_white_1",
        "pawn_white_2",
        "pawn_white_3",
        "pawn_white_4",
        "pawn_white_5",
        "pawn_white_6",
        "pawn_white_7",
        "pawn_white_8",
        "rook_white_1",
        "rook_white_2",
        "knight_white_1",
        "knight_white_2",
        "bishop_white_1",
        "bishop_white_2",
        "queen_white",
        "king_white",
        "pawn_black_1",
        "pawn_black_2",
        "pawn_black_3",
        "pawn_black_4",
        "pawn_black_5",
        "pawn_black_6",
        "pawn_black_7",
        "pawn_black_8",
        "rook_black_1",
        "rook_black_2",
        "knight_black_1",
        "knight_black_2",
        "bishop_black_1",
        "bishop_black_2",
        "queen_black",
        "king_black",
    ]
    
    coord_values = np.array([-0.154, -0.11, -0.066, -0.022, 0.022, 0.066, 0.11, 0.154])
    all_coordinates = np.array(list(product(coord_values, coord_values)))
    np.random.seed(123)
    random.seed(123)
    
    for file_idx in range(1, num_files + 1):
        all_coordinates_idx = np.arange(len(all_coordinates))
        p_skip_piece = 0.8 if (file_idx - 1) <= num_files // 3 else 0.5 if num_files // 3 < (file_idx - 1) <= num_files * 2 // 3 else 0.2
        with (
            open(input_filename, "r") as input_file,
            open(output_dir / f"image{file_idx:05}.world", "w") as world_output_file,
            open(output_dir / f"image{file_idx:05}.txt", "w") as tag_output_file
            ):
            is_pose_line = False
            class_name = None
            for line in input_file.readlines():
                if is_pose_line:
                    is_pose_line = False
                    if random.random() < p_skip_piece:
                        continue
                    rnd_idx = np.random.choice(all_coordinates_idx)
                    all_coordinates_idx = np.delete(all_coordinates_idx, all_coordinates_idx == rnd_idx)
                    x, y = all_coordinates[rnd_idx]
                    line = f"      <pose>{x} {y} 0.815 0 0 0</pose>\n"
                    x, y = x + 0.154, y + 0.154
                    px_coord = (int(395 + y / 0.044 * 70), int(115 + x / 0.044 * 70))
                    tag_line = f"{class_names[class_name]} {px_coord[0]} {px_coord[1]} {dx} {dy}\n"
                    tag_output_file.write(tag_line)
                if any([piece_name in line and "uri" not in line for piece_name in pieces_names]):
                    class_name = line.strip()[13:-2]
                    class_name = class_name[:-2] if len(class_name.split("_")) > 2 else class_name
                    is_pose_line = True
                world_output_file.write(line)


def normalize_labels(dataset_dir: Path) -> None:
    
    for mode_dir in dataset_dir.glob("*"):
        if mode_dir.stem.startswith("."):
            continue
        assert mode_dir.is_dir()
        
        out_dir = mode_dir / "labels"
        out_dir.mkdir(exist_ok=True, parents=True)
        
        for labels_filename in (mode_dir / "labels_original_720_1280").glob("*"):
            if labels_filename.stem.startswith("."):
                continue
            
            with (
                open(labels_filename, "r") as label_file,
                open(out_dir / labels_filename.name, "w") as new_label_file
            ):
                for line in label_file.readlines():
                    class_idx, center_x, center_y, width, height = line.strip().split(" ")
                    normalized_center_x, normalized_width = (int(center_x) - 280) / 720, int(width) / 720
                    normalized_center_y, normalized_height = int(center_y) / 720, int(height) / 720
                    new_label_file.write(f"{class_idx} {normalized_center_x} {normalized_center_y} {normalized_width} {normalized_height}\n")


def crop_and_resize(input_dir: Path, output_dir: Path) -> None:
    
    for image_filename in input_dir.glob("*.jpg"):
        if image_filename.stem.startswith("."):
            continue
        
        image = cv2.imread(str(image_filename))
        ratio = 640 / 720
        resized_image = cv2.resize(image[:, 280:-280, :], (0, 0), fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(output_dir / f"{image_filename.stem}.jpg"), resized_image)


if __name__ == "__main__":
    
    # prepare gazebo world files and label files
    input_filename = Path("/Users/francescobaraldi/Desktop/chess_game_original.world")
    output_dir = Path("/Users/francescobaraldi/Desktop/ros1/chess_dataset")
    num_files = 1000
    generate_world_files(input_filename=input_filename, output_dir=output_dir, num_files=num_files)
    
    # normalize label's coordinates
    dataset_dir = Path("/Users/francescobaraldi/Desktop/chess_dataset")
    normalize_labels(dataset_dir=dataset_dir)
    
    # crop and resize the images
    input_dir = Path("/Users/francescobaraldi/Desktop/ros1/chess_dataset_images_final")
    output_dir = Path("/Users/francescobaraldi/Desktop/chess_dataset_images_resized_640_640")
    output_dir.mkdir(exist_ok=True, parents=True)
    crop_and_resize(input_dir=input_dir, output_dir=output_dir)
