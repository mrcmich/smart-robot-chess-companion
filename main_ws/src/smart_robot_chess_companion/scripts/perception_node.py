#! /usr/bin/env python

from typing import Any
import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image, CompressedImage
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
    input['image'] = message


def perception_node() -> None:
    
    rospy.init_node('perception_script', anonymous=True, log_level=rospy.DEBUG)
    rate = rospy.Rate(2)
    
    bridge = CvBridge()
    
    input = {'is_camera_view_free': False, 'image': None}
    
    camera_view_state_subscriber = rospy.Subscriber(
        'is_camera_view_free', 
        Bool, 
        update_camera_view_state,
        callback_args=[input]
    )
    
    image_raw_subscriber = rospy.Subscriber(
        '/camera/image_raw/compressed', 
        CompressedImage, 
        read_image,
        callback_args=[input]
    )
    
    board_state_publisher = rospy.Publisher('board_state', String, queue_size=10)

    last_board_state_matrix = None
    ratio = 640 / 720
    last_msg_sent = None
    while not rospy.is_shutdown():
        if input['is_camera_view_free'] and input['image'] is not None:
            img = bridge.compressed_imgmsg_to_cv2(input['image'], 'rgb8')
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            resized_img = cv2.resize(img[:, 280:-280, :], (0, 0), fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA)
            board_state_dict, board_state_matrix = predict.predict(model_checkpoint_path=config.MODEL_CHECKPOINT_PATH, img=resized_img, device=config.DEVICE)
            if last_board_state_matrix is None or not np.all(board_state_matrix == last_board_state_matrix):
                last_msg_sent = utils.dict_to_string_ros_message(board_state_dict)
            
            board_state_publisher.publish(last_msg_sent)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        perception_node()
    except rospy.ROSInterruptException:
        pass
