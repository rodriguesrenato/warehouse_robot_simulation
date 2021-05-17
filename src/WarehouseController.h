#ifndef WAREHOUSECONTROLLER_H
#define WAREHOUSECONTROLLER_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include "WarehouseObject.h"
class Product;
class Dispatch;
class Storage;

class WarehouseController : public WarehouseObject
{
public:
    WarehouseController(std::string warehouseControllerName, std::vector<std::unique_ptr<Order>>& orders);
    ~WarehouseController();
    void AddOrder();

private:
    std::string _warehouseControllerName{};
    std::vector<std::unique_ptr<Order>> orders;
};

#endif