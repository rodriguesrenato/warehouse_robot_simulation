#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include <unordered_map>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Model.h"

Model::Model()
{
}

bool Model::Add(std::string modelName, std::string modelPath)
{
    if (Model::_loadedModels.find(modelName) == Model::_loadedModels.end())
    {
        std::string model = Model::ReadModel(modelPath);
        Model::_loadedModels[modelName] = model;
        return true;
    }
    return false;
}
bool Model::Spawn(std::string ObjectName,std::string modelName, geometry_msgs::Pose modelPose)
{
    return GazeboSpawn(ObjectName, Model::_loadedModels[modelName], modelPose);
}
bool Model::Delete(std::string ObjectName)
{
    return GazeboDelete(ObjectName);
}

std::string Model::ReadModel(std::string modelPath)
{
    std::cout << "Start Reading File at " << modelPath << "\n";

    std::ifstream modelFile;
    modelFile.open(modelPath); //open the model file

    std::stringstream strStream;
    strStream << modelFile.rdbuf();      //read the whole file
    std::string model = strStream.str(); // save all content in string format
    std::cout << "Read file" << modelPath << " with size " << model.size() << ".\n";
    return model;
}

bool Model::GazeboSpawn(std::string ObjectName, std::string model, geometry_msgs::Pose initial_pose)
{
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = ObjectName;

    srv.request.initial_pose = initial_pose;

    srv.request.model_xml = model;
    srv.request.reference_frame = "world";

    if (ros::service::call("gazebo/spawn_sdf_model", srv))
    {
        return true;
        std::cout << "Model " << ObjectName << " spawned! \n";
    }
    else
    {
        return false;
        std::cout << "Model " << ObjectName << " not spawned! \n";
    }
}
bool Model::GazeboDelete(std::string ObjectName)
{
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = ObjectName;

    if (ros::service::call("gazebo/delete_model", srv))
    {
        return true;
        std::cout << "Model " << ObjectName << " deleted! \n";
    }
    else
    {
        return false;
        std::cout << "Model " << ObjectName << " not deleted! \n";
    }
}