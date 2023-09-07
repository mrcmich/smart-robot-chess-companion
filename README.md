# Smart Robot Chess Companion (SRCC)

## Introduction
SRCC is a ROS package to be used as a way of simulating a UR3e robot programmed to act as a chess bot against a human player. A Robotiq Hand-E gripper is attached to the robot for all necessary pick-and-place operations. The simulation is run in Gazebo, whereas MoveIt is used for path planning. SRCC relies on a forked version of [cambel's ur3 repository](https://github.com/cambel/ur3).

## Installation
Clone our repository in your preferred directory:
  ```
   git clone https://github.com/mrcmich/smart-robot-chess-companion.git
  ```
Install [rospy_message_converter](https://github.com/DFKI-NI/rospy_message_converter):
  ```
   cd smart-robot-chess-companion/main_ws/src
   git clone https://github.com/DFKI-NI/rospy_message_converter.git
   cd rospy_message_converter
   python setup.py build
   sudo python setup.py install
   cd ../../..
  ```
Install [ur3](https://github.com/mrcmich/ur3/tree/noetic-devel) inside the main catkin workspace of our repository by following [these instructions](https://github.com/mrcmich/ur3/wiki/Compile-from-source).

Finally, run our setup script:
  ```
   python setup.py
  ```

## Simulation
In order to bring up the simulation, run the following command:
  ```
   roslaunch smart-robot-chess-companion chess_game_simulation.launch
  ```
While the simulation is running, it's possible to send commands to joints and gripper of the robot.

## Authors
- [Francesco Baraldi](https://github.com/francescobaraldi)
- [Matteo Pagliani](https://github.com/MatteoPagliani)
- [Me](https://github.com/mrcmich)
