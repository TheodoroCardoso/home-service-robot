#!/bin/sh
xterm  -e  " source devel/setup.bash; roscore" & 
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo view_navigation.launch" &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo keyboard_teleop.launch" 
