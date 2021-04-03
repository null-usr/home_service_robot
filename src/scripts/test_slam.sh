#!/bin/sh
#script to deploy a turtlebot inside the environment, 
#control it with keyboard commands, interface it with a SLAM package, 
#and visualize the map in rviz

#Deploy turtlebot in environment
xterm -e "source ../../devel/setup.bash"
sleep 5
#use find or rospack find instead
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$( rospack find pick_objects )/../map/Apartment_centered.world" &
sleep 10

#Run node to perform SLAM
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 10

#Observe map in rviz
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

#teleop controls
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"