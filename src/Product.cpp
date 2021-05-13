#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include "Product.h"

Product::Product(std::string modelName, std::string modelPath)
{
    _modelName = modelName;
    _modelPath = modelPath;
    std::cout << "Start Read File at " << modelPath << "\n";

    std::ifstream modelFile;
    modelFile.open(modelPath); //open the model file

    std::stringstream strStream;
    strStream << modelFile.rdbuf(); //read the whole file
    _model = strStream.str();       // save all content in string format
    std::cout << "Product created: model" << _modelName << " with size " << _model.size() << ".\n";
}

Product::Product(ProductType productType)
{
    switch (productType)
    {
    case ProductType::productA:
        _modelName = "ProductA" + std::to_string(_id);
        _modelPath = _modelsBasePath + "productA/model.sdf";
        break;

    case ProductType::productB:
        _modelName = "ProductB" + std::to_string(_id);
        _modelPath = _modelsBasePath + "productB/model.sdf";
        break;

    case ProductType::productC:
        _modelName = "ProductC-" + std::to_string(_id);
        _modelPath = modelsBasePath + "productC/model.sdf";
        break;

    default:

        break;
    }

    std::cout << "Start Reading File at " << _modelPath << "\n";

    std::ifstream modelFile;
    modelFile.open(_modelPath); //open the model file

    std::stringstream strStream;
    strStream << modelFile.rdbuf(); //read the whole file
    _model = strStream.str();       // save all content in string format
    std::cout << "Product created: model" << _modelName << " with size " << _model.size() << ".\n";
}

std::string Product::GetName() { return _modelName; }


void Product::Spawn(geometry_msgs::Pose initial_pose)
{
    Product::GazeboSpawn(initial_pose);
    std::cout << "Product " << _modelName << " collected! \n";
}
void Product::Remove()
{
    Product::GazeboDelete();
    std::cout << "Product " << _modelName << " dropped off! \n";
}

void Product::GazeboSpawn(geometry_msgs::Pose initial_pose)
{
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = _modelName;

    srv.request.initial_pose = initial_pose;

    srv.request.model_xml = _model;
    srv.request.reference_frame = "world";

    // if (!gazeboSpawnService->call(srv))
    //     ROS_ERROR("Failed to spawn model ");

    if (ros::service::call("gazebo/spawn_sdf_model", srv))
    {
        std::cout << "Model " << _modelName << " spawned! \n";
    }
    else
    {
        std::cout << "Model " << _modelName << " not spawned! \n";
    }
}
void Product::GazeboDelete()
{
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = _modelName;

    // if (!gazeboDeleteService->call(srv))
    //     ROS_ERROR("Failed to Delete model ");

    if (ros::service::call("gazebo/delete_model", srv))
    {
        std::cout << "Model " << _modelName << " deleted! \n";
    }
    else
    {
        std::cout << "Model " << _modelName << " not deleted! \n";
    }
    // rosservice call gazebo/delete_model '{model_name: rrbot1}'
}