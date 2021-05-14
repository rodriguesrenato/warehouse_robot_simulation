#ifndef PRODUCT_H
#define PRODUCT_H

#include <string>
#include <memory>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

#include "WarehouseObject.h"

class Product : public WarehouseObject
{
public:
    Product(std::string modelName);
    std::string GetName();
    std::string GetModelName();
    bool GetGazeboSpawnStatus();
    void SetGazeboSpawnStatus(bool status);

private:
    std::string _productName{};
    std::string _modelName{};
    bool _isSpawned{false};
};

#endif