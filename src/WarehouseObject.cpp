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

// Common print function protected by a mutex
void WarehouseObject::Print(std::string message)
{
    std::lock_guard<std::mutex> lck(_coutMtx);
    std::cout << "[" << _objectName << "] " << message << std::endl;
}