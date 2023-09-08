import os
import roslaunch
import rospy
import time


if __name__ == "__main__":

    start_file_idx = 61
    num_files = 1
    for file_idx in range(0, 1):
        rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['/home/francesco/smart-robot-chess-companion/main_ws/src/smart-robot-chess-companion/launch/chess_game_simulation.launch', f'world:=image{file_idx:05}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        rospy.loginfo("started")
        time.sleep(60)
        launch.shutdown()
        
        os.system("cd /home/francesco/smart-robot-chess-companion")
        os.system(f"mv /tmp/camera_out/ur3_chess_game_camera_link_camera\(1\)-0000.jpg ./dataset/image{file_idx:05}.jpg")
        os.system("rm /tmp/camera_out/*.jpg")