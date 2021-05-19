#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include "WarehouseObject.h"
#include "Order.h"

Order::Order(std::string orderName)
{
    _orderName = orderName + "#" + std::to_string(_id);
    _objectName = orderName + "#" + std::to_string(_id);
}
Order::~Order()
{
}
void Order::SetOrderName(std::string orderName)
{
    _orderName = orderName;
}
std::string Order::GetOrderName()
{
    return _orderName;
}
void Order::SetRobotWorkerName(std::string robotName)
{
    _robotWorkerName = robotName;
}
std::string Order::GetRobotWorkerName()
{
    return _robotWorkerName;
}
void Order::SetGoalDispatchName(std::string goalDispatchName)
{
    _goalDispatchName = goalDispatchName;
}
std::string Order::GetGoalDispatchName()
{
    return _goalDispatchName;
}

void Order::AddProduct(std::string productName, int quantity)
{
    if (_productsList.find(productName) == _productsList.end())
    {
        _productsList[productName] = quantity;
    }
    else
    {
        _productsList[productName] += quantity; // repeated product are grouped together
    }
}

std::unordered_map<std::string, int> Order::GetProductList()
{
    return _productsList;
}