#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include "ModelController.h"

// Define an unique OrderController name
ModelController::ModelController(std::string modelControllerName)
{
    _objectName = modelControllerName + "#" + std::to_string(_id);
    _type = ObjectType::objectModelController;
}

ModelController::~ModelController()
{
    Print("Shutting down");
}

// Load the model file content and add it to the dictionary of loaded models
bool ModelController::Add(std::string modelName, std::string modelPath)
{
    if (_loadedModels.find(modelName) == _loadedModels.end())
    {
        std::string model = ReadModel(modelPath);
        _loadedModels[modelName] = model;
        return true;
    }
    return false;
}

// Prepare to spawn the object in the Gazebo simulation
bool ModelController::Spawn(std::string ObjectName, std::string modelName, geometry_msgs::Pose modelPose)
{
    // Check if modelName was added to dictionary of models, then call Gazebo Spawn
    if (_loadedModels.find(modelName) != _loadedModels.end())
    {
        return GazeboSpawn(ObjectName, _loadedModels[modelName], modelPose);
    }
    // If modelname wasn't added to dictionary of models, then return false
    return false;
}

// Delete the object in the Gazebo simulation by its unique ObjectName
bool ModelController::Delete(std::string ObjectName)
{
    return GazeboDelete(ObjectName);
}

// Return the whole file content from modelPath
std::string ModelController::ReadModel(std::string modelPath)
{
    // Open the file a modelPath
    std::ifstream modelFile;
    modelFile.open(modelPath);

    // Read the whole file, convert to string format and return it
    std::stringstream strStream;
    strStream << modelFile.rdbuf();
    std::string model = strStream.str();
    
    Print("Filepath " + modelPath + " was read, string size of" + std::to_string(model.size()));
    return model;
}

// Spawn the object in the Gazebo simulation
bool ModelController::GazeboSpawn(std::string ObjectName, std::string model, geometry_msgs::Pose initial_pose)
{
    gazebo_msgs::SpawnModel srv;

    // Set the model name in Gazebo with the unique ObjectName
    srv.request.model_name = ObjectName;

    // Set the pose where this object will be spawned
    srv.request.initial_pose = initial_pose;

    // Set the model_xml with the respective model file content read previously to the dictionary of models
    srv.request.model_xml = model;

    srv.request.reference_frame = "world";

    // Directly call ros service to request an sdf model spawn
    if (ros::service::call("gazebo/spawn_sdf_model", srv))
    {
        Print(ObjectName + " was successfully spawned in Gazebo simulation");
        return true;
    }
    else
    {
        Print(ObjectName + "failed to spawn in Gazebo simulation");
        return false;
    }
}

// Delete the object in the Gazebo simulation by its unique ObjectName
bool ModelController::GazeboDelete(std::string ObjectName)
{
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = ObjectName;

    // Directly call ros service to request an sdf model delete
    if (ros::service::call("gazebo/delete_model", srv))
    {
        Print(ObjectName + " was successfully deleted in Gazebo simulation");
        return true;
    }
    else
    {
        Print(ObjectName + "failed to be deleted in Gazebo simulation");
        return false;
    }
}