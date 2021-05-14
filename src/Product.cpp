#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include "Product.h"

Product::Product(std::string modelName)
{
    _modelName = modelName;
    _productName = modelName + "-" + std::to_string(_id);
}

std::string Product::GetName()
{
    return _productName;
}
std::string Product::GetModelName()
{
    return _modelName;
}
bool Product::GetGazeboSpawnStatus()
{
    return _isSpawned;
}
void Product::SetGazeboSpawnStatus(bool status)
{
    _isSpawned = status;
}
