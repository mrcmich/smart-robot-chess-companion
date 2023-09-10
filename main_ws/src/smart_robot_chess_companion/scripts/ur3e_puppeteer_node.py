#! /usr/bin/env python

import smart_robot_chess_companion.pick_and_place.config as config
from smart_robot_chess_companion.pick_and_place.pick_and_place import RobotChessCompanion
from smart_robot_chess_companion import utils
from ur_control import arm, constants
import rospy
from std_msgs.msg import Bool, String

def extract_commands_from_ros_message(message, *args):
    message_data_dict = utils.dict_from_string_ros_message(message)
    input = args[0][0]
    output = args[0][1]
    n_captured_chess_pieces = input['n_captured_chess_pieces']

    if message_data_dict['capture'] is not None:
        message_data_dict['capture']['final_cell'] = config.MAPPING_N_CAPTURED_CHESS_PIECES_CELL[n_captured_chess_pieces]
        input['n_captured_chess_pieces'] += 1

    cmd = {
        'timestamp': message_data_dict['timestamp'],
        'move': message_data_dict['move'],
        'capture': message_data_dict['capture']
    }
    output.append(cmd)
    
def ur3e_puppeteer_node():
    rospy.init_node('ur3e_puppeteer_node', log_level=rospy.DEBUG)
    rate = rospy.Rate(2)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_chess_companion = RobotChessCompanion(
        robot_arm,
        config.REST_ROBOT_ARM_CONFIGURATION,
        config.RIGHT_ARM_ROBOT_ARM_CONFIGURATION,
        config.LEFT_ARM_ROBOT_ARM_CONFIGURATION,
        config.DEFAULT_GRASP_CONFIGURATIONS_DICT,
        config.OPEN_GRIPPER_POSITION
    )
    is_camera_view_free_publisher = rospy.Publisher('is_camera_view_free', Bool, queue_size=10)
    last_timestamp = None
    input = {'n_captured_chess_pieces': 0}
    output = []
    ur3e_puppeteer_node_cmd_subscriber = rospy.Subscriber(
        'ur3e_puppeteer_node_cmd', 
        String, 
        extract_commands_from_ros_message, 
        callback_args=[input, output],
        queue_size=10
    )
    
    while not rospy.is_shutdown():
        rospy.logdebug(f'command queue: {output}')

        if len(output) == 0:
            is_camera_view_free_publisher.publish(True)
            rate.sleep()
            continue

        cmd = output.pop(0)

        if cmd['timestamp'] is not None and cmd['timestamp'] != last_timestamp:
            is_camera_view_free_publisher.publish(False)
            move_cmd = cmd['move']
            capture_cmd = cmd['capture']

            if capture_cmd is not None:
                robot_chess_companion.move_chess_piece(
                    chess_piece_type=capture_cmd['chess_piece_type'], 
                    starting_cell=capture_cmd['starting_cell'], 
                    final_cell=capture_cmd['final_cell'], 
                )
                rospy.sleep(1.0)

            robot_chess_companion.move_chess_piece(
                chess_piece_type=move_cmd['chess_piece_type'], 
                starting_cell=move_cmd['starting_cell'], 
                final_cell=move_cmd['final_cell'], 
            )
            
            last_timestamp = cmd['timestamp']
            is_camera_view_free_publisher.publish(True)
        else:
            is_camera_view_free_publisher.publish(True)
        
        rate.sleep()
            
if __name__ == '__main__':
    try:
        ur3e_puppeteer_node()
    except rospy.ROSInterruptException:
        pass
