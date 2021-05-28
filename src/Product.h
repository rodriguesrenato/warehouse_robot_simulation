#ifndef PRODUCT_H
#define PRODUCT_H

#include <string>
#include "WarehouseObject.h"

class Product : public WarehouseObject
{
public:
    Product(std::string modelName);
    std::string GetModelName();

private:
    std::string _modelName{};
};

#endif