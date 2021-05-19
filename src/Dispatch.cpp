#include "Dispatch.h"
#include "Product.h"
#include "Model.h"
#include <memory>
#include <string>

Dispatch::Dispatch(std::string modelName, geometry_msgs::Pose dispatchPose, std::shared_ptr<Model> modelController)
{
    _dispatchModelName = modelName;
    _objectName = modelName + "#" + std::to_string(_id);
    _dispatchName = modelName + "#" + std::to_string(_id);
    _dispatchPose = dispatchPose;
    _modelController = modelController;
}
Dispatch::~Dispatch()
{
}

std::string Dispatch::GetName()
{
    return _dispatchName;
}
std::string Dispatch::GetModelName()
{
    return _dispatchModelName;
}
geometry_msgs::Pose Dispatch::GetPose()
{
    return _dispatchPose;
}

bool Dispatch::GetGazeboSpawnStatus()
{
    return _isSpawned;
}
void Dispatch::SetGazeboSpawnStatus(bool status)
{
    _isSpawned = status;
}

bool Dispatch::PickProduct(std::unique_ptr<Product> product)
{
    Print("Picked "+product->GetName());
    return _modelController->Delete(product->GetName());
}