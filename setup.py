from distutils.dir_util import copy_tree
import os
import subprocess

def add_models_to_gazebo_library(): 
    src = 'main_ws/src/smart-robot-chess-companion/models'
    dst = f"{os.environ['HOME']}/.gazebo/models"

    for dir in os.listdir(src):
        copy_tree(f'{src}/{dir}', f'{dst}/{dir}')
        print(f'Copied model {dir} in {dst}.')

def add_source_line_to_bashrc():
    bashrc_line = f'source {os.getcwd()}/main_ws/devel/setup.bash'
    bashrc_path = f"{os.environ['HOME']}/.bashrc"
    cmd = f'echo "{bashrc_line}" >> {bashrc_path}'
    subprocess.run(cmd, shell=True)
    print('Added source line to .bashrc file.')

if __name__ == '__main__':
    add_models_to_gazebo_library()
    add_source_line_to_bashrc()
