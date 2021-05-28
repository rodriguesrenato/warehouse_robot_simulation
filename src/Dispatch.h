#ifndef DISPATCH_H
#define DISPATCH_H

#include <string>
#include <memory>
#include <geometry_msgs/Pose.h>

#include "WarehouseObject.h"
#include "Product.h"
#include "ModelController.h"

// Dispathc Object class
class Dispatch : public WarehouseObject
{
public:
    Dispatch(std::string modelName, geometry_msgs::Pose dispatchPose, geometry_msgs::Pose dispatchPickPose, std::shared_ptr<ModelController> modelController);
    ~Dispatch();
    std::string GetModelName();
    geometry_msgs::Pose GetPose();
    geometry_msgs::Pose GetPickPose();
    bool PickProduct(std::unique_ptr<Product> product);

private:
    std::string _dispatchModelName{};                  // Dispatch model name
    geometry_msgs::Pose _dispatchPose;                 // Dispatch pose in the simulation
    geometry_msgs::Pose _dispatchPickPose;             // Dispatch Product pick up pose in the simulation
    std::shared_ptr<ModelController> _modelController; // ModelController shared pointer to be used to interact with products
};

#endif