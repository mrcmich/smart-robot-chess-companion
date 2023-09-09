#! /usr/bin/env python

import smart_robot_chess_companion.pick_and_place.config as config
from smart_robot_chess_companion.pick_and_place.pick_and_place import move_chess_piece
from smart_robot_chess_companion import utils
from ur_control import arm, constants
import rospy
from std_msgs.msg import Bool, String

def extract_commands_from_ros_message(message, *args):
    message_data_dict = utils.dict_from_string_ros_message(message)
    input = args[0][0]
    output = args[0][1]
    n_captured_chess_pieces = input['n_captured_chess_pieces']
    output['move'] = message_data_dict['move']
    output['capture'] = message_data_dict['capture']

    if output['capture'] is not None:
        output['capture']['final_cell'] = config.MAPPING_N_CAPTURED_CHESS_PIECES_CELL[n_captured_chess_pieces]
        input['n_captured_chess_pieces'] += 1

    output['timestamp'] = message_data_dict['timestamp']
    
def ur3e_puppeteer_node():
    rospy.init_node('ur3e_puppeteer_node')
    rate = rospy.Rate(2)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_arm.set_joint_positions(position=config.REST_ROBOT_ARM_CONFIGURATION, wait=True, t=0.5)
    robot_arm.gripper.command(config.OPEN_GRIPPER_POSITION)
    is_camera_view_free_publisher = rospy.Publisher('is_camera_view_free', Bool, queue_size=10)
    last_timestamp = None
    input = {'n_captured_chess_pieces': 0}
    output = { 
        'timestamp': None , 
        'move': {'chess_piece_type': None, 'starting_cell': None, 'final_cell': None}, 
        'capture': {'chess_piece_type': None, 'starting_cell': None, 'final_cell': None}
    }
    ur3e_puppeteer_node_cmd_subscriber = rospy.Subscriber(
        'ur3e_puppeteer_node_cmd', 
        String, 
        extract_commands_from_ros_message, 
        callback_args=[input, output],
        queue_size=10
    )
    
    while not rospy.is_shutdown():
        if output['timestamp'] is not None and output['timestamp'] != last_timestamp:
            is_camera_view_free_publisher.publish(False)
            move_cmd = output['move']
            capture_cmd = output['capture']

            if capture_cmd is not None:
                move_chess_piece(
                    robot_arm, 
                    chess_piece_type=capture_cmd['chess_piece_type'], 
                    starting_cell=capture_cmd['starting_cell'], 
                    final_cell=capture_cmd['final_cell'], 
                )
                rospy.sleep(0.5)

            move_chess_piece(
                robot_arm, 
                chess_piece_type=move_cmd['chess_piece_type'], 
                starting_cell=move_cmd['starting_cell'], 
                final_cell=move_cmd['final_cell'], 
            )
            
            last_timestamp = output['timestamp']
            is_camera_view_free_publisher.publish(True)
        else:
            is_camera_view_free_publisher.publish(True)
        
        rate.sleep()
            
if __name__ == '__main__':
    try:
        ur3e_puppeteer_node()
    except rospy.ROSInterruptException:
        pass
