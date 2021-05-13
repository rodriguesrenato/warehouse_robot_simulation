#!/bin/sh
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch renato_robot world.launch" &
sleep 10
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch renato_robot amcl.launch" &
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch renato_robot view_navigation.launch" & 
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun add_markers add_markers" & 
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun pick_objects pick_objects" & 
