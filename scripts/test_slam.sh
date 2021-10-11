#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch my_robot view_navigation.launch " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch " & 
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch my_robot mapping.launch" 
