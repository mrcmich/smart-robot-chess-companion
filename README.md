# Smart Robot Chess Companion (SRCC)

## Introduction
SRCC is a ROS package integrating Computer Vision algorithms for the simulation of a collaborative robot, programmed to act as a chess player against the user. The simulation is run in RViz and Gazebo, and processes communicate through ROS messages. Please note that ROS1 Noetic, Gazebo 11 and RViz should have already been installed and configured.

## Setup and usage
Run our Python setup script to load our custom gazebo models in Gazebo's models library, run command *catkin_make* and append line source <srcc-path>/main_ws/devel/setup.bash to file <home>/.bashrc. You can now use smart-robot-chess-companion as any other ROS package. Note that you're not required to run *source devel/setup.bash* when opening a new terminal when using our ROS package.

## Authors
- [Francesco Baraldi](https://github.com/francescobaraldi)
- [Matteo Pagliani](https://github.com/MatteoPagliani)
- [Me](https://github.com/mrcmich)
