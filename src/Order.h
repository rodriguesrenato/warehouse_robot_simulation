#ifndef ORDER_H
#define ORDER_H

#include <string>
#include <unordered_map>

#include "WarehouseObject.h"

class Order : public WarehouseObject
{
public:
    Order(std::string orderName);
    ~Order();
    void SetRobotWorkerName(std::string robotName);
    std::string GetRobotWorkerName();
    void SetGoalDispatchName(std::string goalDispatchName);
    std::string GetGoalDispatchName();

    void AddProduct(std::string productName, int quantity);
    std::unordered_map<std::string, int> GetProductList();

private:
    std::string _robotWorkerName{};                     // Robot name which is working in this order
    std::string _goalDispatchName{};                    // Targeted Dispach name
    std::unordered_map<std::string, int> _productsList; // product list of this order, in form of dictionary of productName:quantity
};

#endif