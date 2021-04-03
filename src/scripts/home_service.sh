#!/bin/sh
#script to deploy a turtlebot inside the environment, 
#visualize the map in rviz, spawn virtual object and have robot go pick it up

#Deploy turtlebot in environment
xterm -e "source ../../devel/setup.bash"
sleep 5
#use find or rospack find instead
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$( rospack find pick_objects )/../map/Apartment_centered.world" &
sleep 10

#Run node to localize robot
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$( rospack find pick_objects )/../map/Apartment.yaml initial_pose_x:=-17 initial_pose_y:=7 initial_pose_a:=-1.57" &
sleep 10

#Observe map in rviz
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

#add_markers node
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 10

#pick_objects node
xterm -e "source ../../devel/setup.bash"
sleep 5
xterm -e "rosrun pick_objects pick_objects"
sleep 10