from distutils.dir_util import copy_tree
import os

def add_models_to_gazebo_library(): 
    src = 'main_ws/src/smart-robot-chess-companion/models'
    dst = f"{os.environ['HOME']}/.gazebo/models"

    for dir in os.listdir(src):
        copy_tree(f'{src}/{dir}', f'{dst}/{dir}')
        print(f'Copied model {dir} in {dst}.')

if __name__ == '__main__':
    add_models_to_gazebo_library()
