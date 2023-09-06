# Smart Robot Chess Companion (SRCC)

## Introduction
SRCC is a ROS package to be used as a way of simulating a UR3e robot programmed to act as a chess bot against a human player. A Robotiq Hand-E gripper is attached to the robot for all necessary pick-and-place operations. The simulation is run in Gazebo, whereas MoveIt is used for path planning. SRCC relies on a forked version of [cambel's ur3 repository](https://github.com/cambel/ur3).

## Installation
Clone this repository, install [ROS packages for the simulation of robot and gripper](https://github.com/mrcmich/ur3/tree/noetic-devel) inside the main_ws folder of the repository and run script setup.py.

## Simulation
In order to bring up the simulation, run the following command:
  ```
   roslaunch smart_robot_chess_companion chess_game_simulation.launch
  ```
While the simulation is running, it's possible to send commands to joints and gripper of the robot.

## Authors
- [Francesco Baraldi](https://github.com/francescobaraldi)
- [Matteo Pagliani](https://github.com/MatteoPagliani)
- [Me](https://github.com/mrcmich)
