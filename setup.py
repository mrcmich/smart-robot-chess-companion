from distutils.dir_util import copy_tree
import os
import subprocess

def set_rwx_file_permissions():
    cmd = 'chmod -R 777 .'
    subprocess.run(cmd, shell=True)
    print('Set all files permissions.')

def catkin_make():
    cmd = 'cd main_ws; catkin_make; cd ..'
    subprocess.run(cmd, shell=True)
    print('Executed command catkin_make.')

def add_source_line_to_bashrc():
    bashrc_line = f'source {os.getcwd()}/main_ws/devel/setup.bash'
    bashrc_path = f"{os.environ['HOME']}/.bashrc"
    cmd = f'echo "{bashrc_line}" >> {bashrc_path}'
    subprocess.run(cmd, shell=True)
    print('Added source line to .bashrc file.')

if __name__ == '__main__':
    set_rwx_file_permissions()
    catkin_make()
    add_source_line_to_bashrc()
