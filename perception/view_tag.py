import cv2


if __name__ == "__main__":
    input_tag_file = "/Users/francescobaraldi/Desktop/chess_dataset/train/labels/image00001.txt"
    input_image = "/Users/francescobaraldi/Desktop/chess_dataset/train/images/image00001.jpg"
    image = cv2.imread(input_image)
    
    with open(input_tag_file, "r") as tag_file:
        for line in tag_file.readlines():
            class_idx, x, y, w, h = line.strip().split(" ")
            x, y, w, h = int(float(x) * 640), int(float(y) * 640), int(float(w) * 640), int(float(h) * 640)
            cv2.rectangle(image, (int(x - w // 2), int(y - h // 2)), (int(x + (w - w // 2)), int(y + (h - h // 2))), color=(0, 0, 255), thickness=2)
            cv2.circle(image, center=(int(x), int(y)), radius=10, color=(0, 0, 255), thickness=-1)
        
        cv2.imwrite("/Users/francescobaraldi/Desktop/image00001_tag.jpg", image)
