from distutils.dir_util import copy_tree
import os

def add_models_to_gazebo_library():
    dst = f"{os.environ['HOME']}/.gazebo/models"

    for dir in os.listdir('main_ws/src/models/'):
        copy_tree(f'main_ws/src/models/{dir}', f'{dst}/{dir}')
        print(f'Copied model {dir} in {dst}.')

if __name__ == '__main__':
    add_models_to_gazebo_library()
