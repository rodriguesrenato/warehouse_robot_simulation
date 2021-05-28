#include "Dispatch.h"

// Define an unique Dispatch name and set class attributes
Dispatch::Dispatch(std::string modelName, geometry_msgs::Pose dispatchPose, geometry_msgs::Pose dispatchPickPose, std::shared_ptr<ModelController> modelController)
{
    _objectName = modelName + "#" + std::to_string(_id);
    _type = ObjectType::objectDispatch;
    _dispatchModelName = modelName;
    _dispatchPose = dispatchPose;
    _dispatchPickPose = dispatchPickPose;
    _modelController = modelController;
}

Dispatch::~Dispatch()
{
    Print("Shutting down");
}

// Return the Dispatch model name
std::string Dispatch::GetModelName()
{
    return _dispatchModelName;
}

// Return the Dispatch pose
geometry_msgs::Pose Dispatch::GetPose()
{
    return _dispatchPose;
}

// Return the Dispatch Product pick up pose
geometry_msgs::Pose Dispatch::GetPickPose()
{
    return _dispatchPickPose;
}

// Pick the requested product and request for delete it from Gazebo simulation
bool Dispatch::PickProduct(std::unique_ptr<Product> product)
{
    Print("Picked " + product->GetName());
    return _modelController->Delete(product->GetName());
}