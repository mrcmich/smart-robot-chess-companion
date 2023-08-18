from distutils.dir_util import copy_tree
import os

def add_models_to_gazebo_library():
    dst = f"{os.environ['HOME']}/.gazebo/models"

    for dir in os.listdir('main_ws/src/models/'):
        dir_name = str(dir).split('/')[-1]
        copy_tree(f'main_ws/src/models/{dir}', f'{dst}/{dir_name}')
        print(f'Copied model {dir_name} in {dst}.')

if __name__ == '__main__':
    add_models_to_gazebo_library()
