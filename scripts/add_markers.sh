#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch renato_robot world.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch renato_robot amcl.launch" &
sleep 3
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch renato_robot view_navigation.launch" & 
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun add_markers add_markers" & 
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosservice call /job_request \"job: 'Add'
pose:
  position:
    x: -0.3
    y: -0.5
    z: 0.2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0\"" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosservice call /job_request \"job: 'Remove'
pose:
  position:
    x: -0.3
    y: -0.5
    z: 0.2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0\"" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosservice call /job_request \"job: 'Add'
pose:
  position:
    x: 9.0
    y: -2.0
    z: 0.2
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0\"" &
