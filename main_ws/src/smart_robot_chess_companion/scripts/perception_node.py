#! /usr/bin/env python

from typing import Any
import cv2
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import smart_robot_chess_companion.utils as utils
import smart_robot_chess_companion.perception.config as config
import smart_robot_chess_companion.perception.predict as predict


def update_camera_view_state(message: Any, *args: Any) -> None:
    input = args[0][0]
    input['is_camera_view_free'] = message.data
    if not message.data:
        input['image'] = None


def read_image(message: Any, *args: Any) -> None:
    input = args[0][0]
    input['image'] = message  # TODO: read correctly the image as np.array


def perception_node() -> None:
    
    rospy.init_node('perception_script', anonymous=True)
    rate = rospy.Rate(1)
    
    bridge = CvBridge()
    
    input = {'is_camera_view_free': False, 'image': None}
    
    camera_view_state_subscriber = rospy.Subscriber(
        'is_camera_view_free', 
        Bool, 
        update_camera_view_state,
        callback_args=[input]
    )
    
    image_raw_subscriber = rospy.Subscriber(
        'camera/image_raw', 
        Image, 
        read_image,
        callback_args=[input]
    )
    
    board_state_publisher = rospy.Publisher('board_state', String, queue_size=10)
    board_state_publisher.publish(True)
    ratio = 640 / 720
    while not rospy.is_shutdown():
        #if input['is_camera_view_free'] and input['image'] is not None:
        if input['image'] is not None:
            print(input['is_camera_view_free'])
            img = bridge.imgmsg_to_cv2(input['image'], 'bgr8')
            resized_img = cv2.resize(img[:, 280:-280, :], (0, 0), fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
            board_state_dict, board_state_matrix = predict.predict(model_checkpoint_path=config.MODEL_CHECKPOINT_PATH, img=resized_img, device='cpu')
            # TODO: create board state and check if it has changed
            # TODO: send the board state to paglia if it has changed
            pass
        
        rate.sleep()


if __name__ == '__main__':
    try:
        perception_node()
    except rospy.ROSInterruptException:
        pass
