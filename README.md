# robot_motion_opt_tesseract

## Setup Instructions

Create your workspace directory and src directory by running
```
mkdir -p catkin_ws/src
```
Copy this package and it's contents into the created src directory

From the main workspace level, run (note this will take a while to clone down all the repositories)
```
wstool init src src/dependencies.rosinstall
```
*If you do not have wstool installed, see [here](http://wiki.ros.org/wstool)

After wstool finishes cloning all the repositories, from the same directory in a terminal run
```
rosdep install --from-paths src --ignore-src -r -y
```
*If you do not have rosdep installed, see [here](http://wiki.ros.org/rosdep)

Now source ROS Noetic and build
```
source /opt/ros/noetic/setup.bash
catkin build
```
