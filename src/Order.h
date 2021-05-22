#ifndef ORDER_H
#define ORDER_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include "WarehouseObject.h"
class Product;
class Dispatch;
class Storage;

class Order : public WarehouseObject
{
public:
    Order(std::string orderName);
    ~Order();
    void SetName(std::string orderName);
    std::string GetName();
    void SetRobotWorkerName(std::string robotName);
    std::string GetRobotWorkerName();
    void SetGoalDispatchName(std::string goalDispatchName);
    std::string GetGoalDispatchName();
    
    void AddProduct(std::string productName, int quantity);
    std::unordered_map<std::string,int> GetProductList();
private:
    std::string _orderName{};
    std::string _robotWorkerName{};                         // robotName who is working in this order
    std::string _goalDispatchName{};
    std::unordered_map<std::string, int> _productsList; // dictionary of productName:quantity
};

#endif