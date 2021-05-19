#include <algorithm>
#include "WarehouseObject.h"

int WarehouseObject::_totalObjects = 0;
std::mutex WarehouseObject::_coutMtx; // TODO: Check if its necessary

WarehouseObject::WarehouseObject()
{
    _type = ObjectType::noObject;
    _id = _totalObjects++;
}

WarehouseObject::~WarehouseObject()
{
    // set up thread barrier before this object is destroyed
    std::for_each(threads.begin(), threads.end(), [](std::thread &t) {
        t.join();
    });
}

void WarehouseObject::Print(std::string message)
{
    std::lock_guard<std::mutex> lck(_coutMtx);
    std::cout << "[" << _objectName << "] " << message << std::endl;
}