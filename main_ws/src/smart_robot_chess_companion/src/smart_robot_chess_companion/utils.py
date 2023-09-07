from rospy_message_converter import message_converter
import json
from datetime import datetime

def dict_to_string_ros_message(dictionary):
    dictionary['timestamp'] = str(datetime.now())
    payload = json.dumps(dictionary)
    data = {'data': payload}
    message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', data)
    return message

def dict_from_string_ros_message(message):
    payload = message.data
    dictionary = json.loads(payload)
    return dictionary
