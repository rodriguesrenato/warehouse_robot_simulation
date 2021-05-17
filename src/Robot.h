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

#include "WarehouseObject.h"
#include "Order.h"
class Product;
class Model;
class Dispatch;
class Storage;


class Robot : public WarehouseObject, public std::enable_shared_from_this<Robot>
{
public:
    Robot(std::string robotName);
    ~Robot();
    void SetOrder(Order order);
    bool isStandby();
    void ExecuteOrder();
private:
    std::string _robotName{};
    geometry_msgs::Pose _standbyPose; // inital and standby pose to wait new order
    Order _order;
    std::vector<std::unique_ptr<Product>> _takenProducts;

};

#endif