#include "ros/ros.h"

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <ios>
#include <math.h>


#include "Model.h"
#include "Storage.h"

std::shared_ptr<Model> models = std::make_shared<Model>();//"box1","/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");

void remove_object_callback(const std_msgs::String &str)
{
    std::cout << "remove_object_callback\n";
    bool res = models->Delete(str.data);
    std::cout << "res: " << res << std::endl;
}

// 
void spawn_object_callback(const std_msgs::String &str)
{
    std::cout << "spawn_object_callback\n";
    geometry_msgs::Pose pose;
    pose.position.x = 1;
    pose.position.y = 1;
    pose.position.z = 0.25; 
    
    bool res = models->Spawn(str.data,"b",pose);
    std::cout << "res: " << res << std::endl;
    
}

int main(int argc, char **argv)
{
    // Initialize the WarehouseSimulation node and create a handle to it
    ros::init(argc, argv, "WarehouseSimulation");
    ros::NodeHandle n;

    // Load models
    models->Add("a","/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    models->Add("b","/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    models->Add("c","/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");

    // Create Storage objects
    // std::vector<std::shared_ptr<Storage>> storages;

    // std::for_each(storages.begin(), storages.end(), [](std::shared_ptr<Storage> &s) {
    //     s->Simulate();
    // });


    ros::Subscriber sub1 = n.subscribe("test/1", 10, spawn_object_callback);
    ros::Subscriber sub2 = n.subscribe("test/2", 10, remove_object_callback);
    

    ros::spin();

    return 0;
} 
