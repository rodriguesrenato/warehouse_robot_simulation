#include "Product.h"

// Define an unique Product name and set the model name
Product::Product(std::string modelName)
{
    _objectName = modelName + "#" + std::to_string(_id);
    _type = ObjectType::objectProduct;
    _modelName = modelName;
}

// Return the model name
std::string Product::GetModelName()
{
    return _modelName;
}
