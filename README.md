# robot_motion_opt_tesseract

## Setup Instructions

Create your workspace directory and src directory by running
```
mkdir -p catkin_ws/src
```
Copy this repository and it's contents into the created src directory

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

## Run Application

Source your workspace and launch the application.
```
source devel/setup.bash
roslaunch robot_motion_planning application_setup.launch
```
*To use a different csv file than the default, add `csv_filepath:=path/to/csv/file` to the end of the above command

Open a new terminal to start the planning service.
```
source devel/setup.bash
rosservice call /plan_process "{}"
```

Motion planning parameters can be found and edited at `robot_motion_planning/config/planner_config.yaml`.
