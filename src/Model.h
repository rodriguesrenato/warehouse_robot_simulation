#ifndef MODEL_H
#define MODEL_H

#include <unordered_map>
#include <string>
#include <memory>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <iostream>


class Model
{
public:
    Model();
    bool Add(std::string modelName, std::string modelPath);
    bool Spawn(std::string ObjectName,std::string modelName,geometry_msgs::Pose modelPose);
    bool Delete(std::string ObjectName);

private:
    std::unordered_map <std::string,std::string> _loadedModels; // dictionary of modelName:model
    bool GazeboSpawn(std::string ObjectName, std::string model, geometry_msgs::Pose initial_pose);
    bool GazeboDelete(std::string ObjectName);
    std::string ReadModel(std::string modelPath);
    
};

#endif