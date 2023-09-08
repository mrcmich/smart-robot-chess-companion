from pathlib import Path
import os
import roslaunch
import rospy
import time


def extract_images_from_gazebo_simulation(start_file_idx: int, num_files: int, launch_filename: Path, output_dir: Path) -> None:

    for file_idx in range(start_file_idx, start_file_idx + num_files):
        rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = [str(launch_filename), f'world:=image{file_idx:05}']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        rospy.loginfo("started")
        time.sleep(60)
        launch.shutdown()
        
        os.system(f"mv /tmp/camera_out/ur3_chess_game_camera_link_camera\(1\)-0000.jpg {str(output_dir / f'image{file_idx:05}.jpg')}")
        os.system("rm /tmp/camera_out/*.jpg")


if __name__ == "__main__":
    
    start_file_idx = 1
    num_files = 100
    launch_filename = Path('/home/francesco/smart-robot-chess-companion/main_ws/src/smart-robot-chess-companion/launch/chess_game_simulation.launch')
    output_dir = Path('/home/francesco/smart-robot-chess-companion/dataset')
    output_dir.mkdir(exist_ok=True, parents=True)
    
    extract_images_from_gazebo_simulation(start_file_idx=start_file_idx, num_files=num_files, launch_filename=launch_filename, output_dir=output_dir)
