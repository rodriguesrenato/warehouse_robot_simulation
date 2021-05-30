#!/bin/sh
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch warehouse_robot_simulation world.launch" &
sleep 6
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch warehouse_robot_simulation robot_spawner.launch" &
sleep 6
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch warehouse_robot_simulation amcl.launch" &
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch warehouse_robot_simulation view_navigation.launch" & 
sleep 5
xterm  -e  " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch warehouse_robot_simulation warehouse_simulation.launch" & 

# rostopic pub /warehouse/order/add std_msgs/String "data: 'DispatchA ProductR 6 ProductG 4'"