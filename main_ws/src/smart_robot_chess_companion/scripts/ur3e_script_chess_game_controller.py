#! /usr/bin/env python

import smart_robot_chess_companion.pick_and_place.config as config
from smart_robot_chess_companion.pick_and_place.pick_and_place import move_chess_piece
from smart_robot_chess_companion import utils
from ur_control import arm, constants
import rospy
from std_msgs.msg import Bool, String

def execute_move_chess_piece_command(command, *args):
    command_data_dict = utils.dict_from_string_ros_message(command)
    input = args[0][0]
    output = args[0][1]
    n_captured_chess_pieces = input['n_captured_chess_pieces']
    chess_piece_type = command_data_dict['chess_piece_type']
    starting_cell = command_data_dict['starting_cell']

    if command_data_dict['final_cell'] is None:
        final_cell = config.MAPPING_N_CAPTURED_CHESS_PIECES_CELL[n_captured_chess_pieces]
    else:
        final_cell = command_data_dict['final_cell']

    output['timestamp'] = command_data_dict['timestamp']
    output['chess_piece_type'] = chess_piece_type
    output['starting_cell'] = starting_cell
    output['final_cell'] = final_cell
    
def ur3e_script_chess_game_controller():
    rospy.init_node('ur3e_script_control')
    rate = rospy.Rate(2)
    robot_arm = arm.Arm(ft_sensor=True, gripper=constants.GENERIC_GRIPPER) 
    robot_arm.set_joint_positions(position=config.REST_ROBOT_ARM_CONFIGURATION, wait=True, t=0.5)
    robot_arm.gripper.command(config.OPEN_GRIPPER_POSITION)
    camera_view_state_publisher = rospy.Publisher('is_camera_view_free', Bool, queue_size=10)
    last_timestamp = None
    input = { 'n_captured_chess_pieces': 0 }
    output = {'timestamp': None, 'chess_piece_type': None, 'starting_cell': None, 'final_cell': None}
    move_command_subscriber = rospy.Subscriber(
        'move_chess_piece_cmd', 
        String, 
        execute_move_chess_piece_command, 
        callback_args=[input, output]
    )
    
    while not rospy.is_shutdown():
        if output['timestamp'] is not None and output['timestamp'] != last_timestamp:
            camera_view_state_publisher.publish(False)
            move_chess_piece(
                robot_arm, 
                chess_piece_type=output['chess_piece_type'], 
                starting_cell=output['starting_cell'], 
                final_cell=output['final_cell'], 
            )
            input['n_captured_chess_pieces'] += 1
            last_timestamp = output['timestamp']
            camera_view_state_publisher.publish(True)
        else:
            camera_view_state_publisher.publish(True)
        
        rate.sleep()
            
if __name__ == '__main__':
    try:
        ur3e_script_chess_game_controller()
    except rospy.ROSInterruptException:
        pass
