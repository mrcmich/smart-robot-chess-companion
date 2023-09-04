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
    
def srcc_setup():
    cwd = os.getcwd()
    cmd = f'cd main_ws/src/smart_robot_chess_companion; python setup.py build; sudo python setup.py install; cd {cwd}'
    subprocess.run(cmd, shell=True)
    print('Executed smart_robot_chess_companion ros package setup script.')

if __name__ == '__main__':
    set_rwx_file_permissions()
    srcc_setup()
    catkin_make()
    add_source_line_to_bashrc()
