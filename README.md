# Smart Robot Chess Companion (SRCC)

## Introduction
SRCC is a ROS package integrating Computer Vision algorithms for the simulation of a UR3e collaborative robot, programmed to act as a chess player against the user. A Robotiq Hand-E gripper is attached to the robot for all necessary pick-and-place operations. The simulation is run in Gazebo, whereas MoveIt is used for path planning.

## Installation
Execute our Python setup script to load our custom gazebo models in Gazebo's models library, run command *catkin_make* and append line source <srcc-path>/main_ws/devel/setup.bash to file <home>/.bashrc. You can now use smart-robot-chess-companion as any other ROS package. Note that you're not required to run *source devel/setup.bash* when opening a new terminal when using our ROS package.

# Simulation
In order to bring up the simulation, run the following command:
  ```
   roslaunch smart-robot-chess-companion chess_game_simulation.launch
  ```
While the simulation is running, it's possible to send commands to joints and gripper of the robot.

## Authors
- [Francesco Baraldi](https://github.com/francescobaraldi)
- [Matteo Pagliani](https://github.com/MatteoPagliani)
- [Me](https://github.com/mrcmich)
