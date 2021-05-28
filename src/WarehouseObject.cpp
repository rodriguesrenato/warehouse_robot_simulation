#include <algorithm>
#include <iostream>
#include "WarehouseObject.h"

// initialize static variables
int WarehouseObject::_totalObjects = 0;
std::mutex WarehouseObject::_coutMtx;

WarehouseObject::WarehouseObject()
{
    _type = ObjectType::noObject;
    _id = _totalObjects++;
}

WarehouseObject::~WarehouseObject()
{
    // Join all threads stored to make a thread barrier before this object is destroyed
    std::for_each(threads.begin(), threads.end(), [](std::thread &t) {
        t.join();
    });
}

// Return the Object name
std::string WarehouseObject::GetName()
{
    return _objectName;
}

// Return the Object type
ObjectType WarehouseObject::GetType()
{
    return _type;
}

// Common print function protected by a mutex
void WarehouseObject::Print(std::string message)
{
    std::lock_guard<std::mutex> lck(_coutMtx);

    // Using cout directly instead of ROS printing functions
    std::cout << "[" << _objectName << "] " << message << std::endl;
}