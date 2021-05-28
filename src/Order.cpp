#include "Order.h"

// Define an unique Order name
Order::Order(std::string orderName)
{
    _objectName = orderName + "#" + std::to_string(_id);
}

Order::~Order()
{
    Print("Order closed");
}

// Set the robot name that will execute this order
void Order::SetRobotWorkerName(std::string robotName)
{
    _robotWorkerName = robotName;
}

// Return the robot name that is executing this order
std::string Order::GetRobotWorkerName()
{
    return _robotWorkerName;
}

// Set the targeted Dispatch name
void Order::SetGoalDispatchName(std::string goalDispatchName)
{
    _goalDispatchName = goalDispatchName;
}

// Return the targeted Dispatch name
std::string Order::GetGoalDispatchName()
{
    return _goalDispatchName;
}

// Add a product to this Order product list
void Order::AddProduct(std::string productName, int quantity)
{
    if (_productsList.find(productName) == _productsList.end())
    {
        _productsList[productName] = quantity;
    }
    else
    {
        // if same product was previouly added, then just sum quantities
        _productsList[productName] += quantity; 
    }
}

// Return the full product list
std::unordered_map<std::string, int> Order::GetProductList()
{
    return _productsList;
}