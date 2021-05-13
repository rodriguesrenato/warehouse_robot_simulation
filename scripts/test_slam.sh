#!/bin/sh
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch  delivery_robot_simulation world.launch" &
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch  delivery_robot_simulation robot_spawner.launch" &
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py" & 
sleep 3
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch  delivery_robot_simulation slam_gmapping.launch " & 
sleep 3
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch delivery_robot_simulation view_navigation.launch" & 
# sleep 5
# remider: to save the generated map, run: rosrun map_server map_saver -f renato_restaurant