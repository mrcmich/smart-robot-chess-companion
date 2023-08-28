# robotiq

ROS metapackage based on the package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg).

## Setup

  * Robotiq 85 Gripper with K-1363 Controller. (Modbus TCP/IP)
  * Robotiq Hand-e (Universal Robots e-series)

## Maintainer

[Cristian Beltran](cristianbehe.me)

## Documentation

  * See the installation instructions below.
  * Throughout the various files in this repository.

## Installation

see [ur3-repo](https://github.com/cambel/ur3) for a complete example using the UR3e robot in Gazebo simulator.

Go to your ROS working directory. e.g.
```{bash}
cd ~/catkin_ws/src
```

Clone these repository:
```{bash}
git clone https://github.com/cambel/robotiq.git
```

Install any missing dependencies using rosdep:
```{bash}
rosdep update
rosdep install --from-paths . --ignore-src -y
```

Now compile your ROS workspace. e.g.
```{bash}
cd ~/catkin_ws && catkin_make
```

### Testing the Installation

Be sure to always source the appropriate ROS setup file, e.g:
```{bash}
source ~/catkin_ws/devel/setup.bash
```
You might want to add that line to your `~/.bashrc`

Try the `cmodel_simple_controller`:
```{bash}
roslaunch robotiq_control cmodel_simple_controller.launch ip:=ROBOTIQ_IP_ADDRESS
```
Expected output:
```
Simple C-Model Controller
-----
Current command:  rACT = 0, rGTO = 0, rATR = 0, rPR = 0, rSP = 0, rFR = 0
-----
Available commands

r: Reset
a: Activate
c: Close
o: Open
(0-255): Go to that position
f: Faster
l: Slower
i: Increase force
d: Decrease force
-->
```
