#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <unordered_map>
#include <deque>

#include "WarehouseObject.h"
// #include "Order.h"
#include "Storage.h"
#include "Product.h"
#include "Order.h"
#include "Dispatch.h"
#include "OrderController.h"

// make a status variable to check if robot is standby
enum StatusType{
    standby,
    requestOrder,
    move,
    requestProducts,
    dispatchOrder,
};

class Robot : public WarehouseObject
{
public:
    Robot(std::string robotName,std::vector<std::shared_ptr<Storage>> &storages,std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<OrderController> orderController);
    ~Robot();
    void SetStatus(StatusType status);
    StatusType GetStatus();
    void SetOrder(Order order);
    void SetWarehouseObjects(std::vector<std::shared_ptr<Storage>> storages,std::vector<std::shared_ptr<Dispatch>> dispatches);
    bool isStandby();
    void ExecuteOrder();
    void StartOperation();
    std::deque<std::shared_ptr<Storage>> GetStoragesToGo();

private:
    std::string _robotName{};
    
    geometry_msgs::Pose _standbyPose; // inital and standby pose to wait new order
    std::shared_ptr<Order> _order;
    StatusType _status{StatusType::standby};
    
    std::vector<std::unique_ptr<Product>> _takenProducts;
    std::vector<std::shared_ptr<Storage>> _storages;
    std::vector<std::shared_ptr<Dispatch>> _dispatches;
    std::shared_ptr<OrderController> _orderController;
    std::vector<std::thread> _threads;
};

#endif