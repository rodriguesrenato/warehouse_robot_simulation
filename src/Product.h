#ifndef PRODUCT_H
#define PRODUCT_H

#include <string>
#include <memory>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

#include "WarehouseObject.h"

enum ProductType
{
    noProduct,
    productA,
    productB,
    productC,
};


class Product : public WarehouseObject
{
public:
    Product(std::string modelName,std::shared_ptr<std::string> model);
    std::string GetName();

private:
    std::string _productName;
    std::string _modelName;
    std::shared_ptr<std::string> _model;
    bool isSpawned{false};
};

#endif