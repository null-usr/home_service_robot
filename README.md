# Udacity Home Service Robot

Final home service robot project for the Udacity Robotics Software Engineer Nanodegree Program

## Prerequisites

* Gazebo >= 7.0
* ROS Kinetic

## Directory Tree
```
├───README.md
├───add_markers
│   ├───include
│   │   └───add_markers
│   └───src
├───add_markers_original
│   ├───include
│   │   └───add_markers_original
│   └───src
├───map
├───pick_objects
│   ├───include
│   │   └───pick_objects
│   ├───src
│   └───srv
├───res
├───rvizConfig
├───scripts
├───slam_gmapping
│   ├───gmapping
│   └───slam_gmapping
├───turtlebot
│   ├───turtlebot
│   ├─── ... 
│   └───turtlebot_teleop
├───turtlebot_interactions
│   ├───turtlebot_dashboard
│   ├─── ...
│   └───turtlebot_rviz_launchers
└───turtlebot_simulator
    ├───turtlebot_gazebo
    └─── ...
```
* rvizConfig contains the configured rviz view for the project
* slam_gmapping, turtlebot, turglebot_interaction and turtlebot_simulator are official ROS packages
* scripts contains test and run scripts for the project

## Build

This repository only represents the ROS package, therefore you need to create a catkin workspace.

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/null-usr/home_service_robot.git
```

Install dependencies

```
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
```

Then build the workspace

```
cd ..
catkin_make
```

## Run Scripts

```
cd src/scripts
```

From here, you can run
* > ./test_slam.sh
  * to test SLAM
* > ./test_localization.sh
  * to test localization
* > ./pick_objects.sh
  * to test robot movement to object locations
* > ./add_markers.sh
  * to test adding markers to the rviz scene
* > ./home_service.sh
  * to launch the full simulation of a robot moving to an object, picking it up, then dropping it elsewhere
