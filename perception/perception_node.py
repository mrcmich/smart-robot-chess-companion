import cv2
import numpy as np
import rospy
from std_msgs.msg import String, Image

from ros_publisher import ROSPublisher


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listen():    
    rospy.init_node('listener_node', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, callback)
    rospy.spin()


def main():
    try:
        rospy.loginfo("Started subscriber_node, now listening to messages...")
        listen()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
