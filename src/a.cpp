#include "ros/ros.h"

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <ios>
#include <math.h>

// Define a global client that can request services
ros::ServiceClient spawnSrv;
ros::ServiceClient removeSrv;

static std::string ReadFile(const std::string &path)
{   
    std::ifstream inFile;
    inFile.open(path); //open the input file

    std::stringstream strStream;
    strStream << inFile.rdbuf(); //read the file
    std::string str = strStream.str(); //str holds the content of the file
    std::cout << str << "\n";

    return str;
}

// void remove_object_callback(const std_msgs::String &str)
// {
//     gazebo_msgs::SpawnModel srv;
//     srv.request.model_name = "Coke_can_"+str.data;

//     if (!removeSrv.call(srv))
//         ROS_ERROR("Failed to call service command_robot");
// }

// 
void spawn_object_callback(const std_msgs::String &str)
{
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = "Coke_can_"+str.data;
    
    geometry_msgs::Pose pose;
    pose.position.x = 1;
    pose.position.y = 1;
    pose.position.z = 0.25;
    srv.request.initial_pose = pose;

    srv.request.model_xml = ReadFile("/home/renato/catkin_ws/src/deliver_robot_simulation/models/box/model.sdf");
    srv.request.reference_frame = "world";

    if (!spawnSrv.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

int main(int argc, char **argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "GazeboSpawnObject");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    // spawnSrv = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    // removeSrv = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/delete_model");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the spawn_object_callback function
    ros::Subscriber sub1 = n.subscribe("test/1", 10, spawn_object_callback);
    // ros::Subscriber sub2 = n.subscribe("test/2", 10, remove_object_callback);
    // std::string ss = "test";
    // spawn_object_callback(std::move(ss));
    // Handle ROS communication events
    ros::spin();

    return 0;
} 

/*

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 1

f = open('/home/benb/Dropbox/RoboticsResearch/WAMInProgress/tp/AllInOne/Trajectory_Phonebook/ros_stuff/src/iiim_wam_description/wam.sdf','r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("some_robo_name", sdff, "robotos_name_space", initial_pose, "world")
*/