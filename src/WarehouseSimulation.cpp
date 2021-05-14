#include "ros/ros.h"

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <ios>
#include <math.h>
#include <memory>
#include <thread>
#include <mutex>
#include <string>

#include "Model.h"
#include "Storage.h"

std::shared_ptr<Model> modelsG = std::make_shared<Model>(); //"box1","/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");

void remove_object_callback(const std_msgs::String &str)
{
    std::cout << "remove_object_callback\n";
    bool res = modelsG->Delete(str.data);
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

    bool res = modelsG->Spawn(str.data, "b", pose);
    std::cout << "res: " << res << std::endl;
}

void loadModels(std::shared_ptr<Model> &models)
{

}

int main(int argc, char **argv)
{
    // Initialize the WarehouseSimulation node and create a handle to it
    ros::init(argc, argv, "WarehouseSimulation");
    ros::NodeHandle n;
    
    std::shared_ptr<Model> models = std::make_shared<Model>();
    // Load models
    models->Add("productA", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    models->Add("storageA", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    models->Add("dropZoneA", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    modelsG->Add("a", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    modelsG->Add("b", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    modelsG->Add("c", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/box/model.sdf");
    
    // Create Storage objects
    std::vector<std::shared_ptr<Storage>> storages;

    geometry_msgs::Pose storagePose;
    storagePose.position.x = 0.0;
    storagePose.position.y = -2.0;
    storagePose.position.z = 1.5;

    geometry_msgs::Pose productOutputPose;
    productOutputPose.position.x = 0.0;
    productOutputPose.position.y = -2.0;
    productOutputPose.position.z = 0.6;

    Storage storage1("storageA",storagePose,productOutputPose,models);
    // storages.push_back(storage1);
    storages.push_back(std::make_shared<Storage>("storageA",storagePose,productOutputPose,models));
    std::for_each(storages.begin(), storages.end(), [](std::shared_ptr<Storage> &s) {
        s->SetProductionModel("productA");
        s->Simulate();
    });

    ros::Subscriber sub1 = n.subscribe("test/1", 10, &Storage::RequestProduct1,&*storages[0]);
    ros::Subscriber sub2 = n.subscribe("test/2", 10, remove_object_callback);

    ros::spin();

    return 0;
}
