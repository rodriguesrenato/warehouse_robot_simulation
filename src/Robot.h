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

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// make a status variable to check if robot is standby
enum StatusType
{
    offline,
    standby,
    requestOrder,
    processOrder,
    move,
    requestProducts,
    dispatchOrder,
};

class Robot : public WarehouseObject
{
public:
    Robot(std::string robotName, std::string actionName, std::vector<std::shared_ptr<Storage>> &storages, std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<OrderController> orderController);
    ~Robot();
    void SetStatus(StatusType status);
    StatusType GetStatus();
    void SetOrder(Order order);
    void SetWarehouseObjects(std::vector<std::shared_ptr<Storage>> storages, std::vector<std::shared_ptr<Dispatch>> dispatches);
    bool isStandby();
    void ExecuteOrder();
    void StartOperation();
    std::deque<std::shared_ptr<Storage>> GetStoragesToGo();
    bool Move(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac, geometry_msgs::Pose goal);

private:
    std::string _robotName{};
    std::string _actionName{};

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