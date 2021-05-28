#ifndef MODEL_H
#define MODEL_H

#include <unordered_map>
#include <string>
#include <geometry_msgs/Pose.h>
#include "WarehouseObject.h"

// ModelController class is responsible to interact with Gazebo simulation and store models XML file content
class ModelController : public WarehouseObject
{
public:
    ModelController(std::string modelControllerName);
    ~ModelController();
    bool Add(std::string modelName, std::string modelPath);
    bool Spawn(std::string ObjectName, std::string modelName, geometry_msgs::Pose modelPose);
    bool Delete(std::string ObjectName);

private:
    std::unordered_map<std::string, std::string> _loadedModels{}; // dictionary of modelName:modelXML_content
    bool GazeboSpawn(std::string ObjectName, std::string model, geometry_msgs::Pose initial_pose);
    bool GazeboDelete(std::string ObjectName);
    std::string ReadModel(std::string modelPath);
};

#endif