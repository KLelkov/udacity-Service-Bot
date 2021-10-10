#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; roslaunch my_robot view_navigation.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; rosrun add_markers markers_srv" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source /home/robond/catkin_service/devel/setup.bash; rosrun pick_objects home_service" 
