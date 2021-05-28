#ifndef WAREHOUSEOBJECT_H
#define WAREHOUSEOBJECT_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>

// Warehouse objects type
enum ObjectType
{
    noObject,
    objectRobot,
    objectStorage,
    objectProduct,
    objectDispatch,
    objectOrder,
    objectOrderController,
    objectModelController,
};

// Parent class for all objects in the warehouse simulation
class WarehouseObject
{
public:
    WarehouseObject();
    ~WarehouseObject();
    std::string GetName();
    ObjectType GetType();
    void Print(std::string message); // Common print function protected by a mutex

protected:
    ObjectType _type;                 // Object type defined
    int _id;                          // Unique id generated
    std::string _objectName;          // Unique object name
    std::vector<std::thread> threads; // Vector of threads created
    static std::mutex _coutMtx;       // Print function mutex

private:
    static int _totalObjects; // Global counter for generate unique ids
};

#endif