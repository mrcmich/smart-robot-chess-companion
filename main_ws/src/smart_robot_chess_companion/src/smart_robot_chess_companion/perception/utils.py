from argparse import ArgumentParser, Namespace


def parse_arguments() -> Namespace:
    
    parser = ArgumentParser()
    parser.add_argument('--yolo_version', type=str, default='yolov8n', choices=['yolov8n', 'yolov8s', 'yolov8m', 'yolov8l', 'yolov8x'])
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch_size', type=int, default=16)
    parser.add_argument('--num_workers', type=int, default=0)
    parser.add_argument('--image_size_h', type=int, default=640)
    parser.add_argument('--image_size_w', type=int, default=640)
    parser.add_argument('--checkpoint_period', type=int, default=1)
    parser.add_argument('--lr', type=float, default=1e-4)
    parser.add_argument('--optimizer', type=str, default='Adam', choices=['Adam', 'SGD'])
    parser.add_argument('--device', type=str, default="cpu")
    parser.add_argument('--dataset_filename', type=str, default="chess.yaml")
    parser.add_argument('--checkpoint_path', type=str, default="/Users/francescobaraldi/Desktop/checkpoint.pt")

    args = parser.parse_args()
    return args
