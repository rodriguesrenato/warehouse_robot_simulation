#ifndef WAREHOUSEOBJECT_H
#define WAREHOUSEOBJECT_H


#include <mutex>
#include <thread>
#include <vector>

#include "Model.h"

enum ObjectType
{
    noObject,
    objectRobot,
    objectStorage,
    objectProduct,
    objectDropZone,
};

class WarehouseObject
{
public:
    WarehouseObject();
    ~WarehouseObject();
    int getID() { return _id; }
    ObjectType getType() { return _type; }
    void Print(std::string message);

protected:
    ObjectType _type;
    int _id;
    std::string _objectName;
    std::vector<std::thread> threads;
    static std::mutex _coutMtx;

private:
    static int _totalObjects;
};

#endif