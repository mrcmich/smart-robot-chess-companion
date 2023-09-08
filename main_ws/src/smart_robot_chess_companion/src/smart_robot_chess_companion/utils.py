from rospy_message_converter.message_converter import convert_dictionary_to_ros_message
import json
from datetime import datetime

def dict_to_string_ros_message(dictionary):
    dictionary['timestamp'] = str(datetime.now())
    payload = json.dumps(dictionary)
    data = {'data': payload}
    message = convert_dictionary_to_ros_message('std_msgs/String', data)
    return message

def dict_from_string_ros_message(message):
    payload = message.data
    dictionary = json.loads(payload)
    return dictionary
