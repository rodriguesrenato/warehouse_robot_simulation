#ifndef ORDERCONTROLLER_H
#define ORDERCONTROLLER_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <deque>
#include <condition_variable>
#include <list>
#include <unordered_map>
#include "WarehouseObject.h"
class Order;


class OrderController : public WarehouseObject
{
public:
    OrderController(std::string orderControllerName);
    ~OrderController();
    void AddOrder(const std_msgs::String &str);
    std::shared_ptr<Order> RequestNextOrder(std::string robotName);
    std::shared_ptr<Order> RequestNextOrderWithTimeout(std::string robotName,int timeout);
    void CloseOrder(std::shared_ptr<Order> order);

private:
    // std::unordered_map<std::shared_ptr<Robot>, std::shared_ptr<Order>> _robotsStatus;

    std::deque<std::shared_ptr<Order>> _queue;
    std::condition_variable _queueCond;
    std::mutex _queueMtx;

    // std::mutex _orderControllerMtx;

    std::string _orderControllerName{};
    std::vector<std::shared_ptr<Order>> _orders;
    std::vector<std::thread> _threads;
    // std::shared_ptr<MessageQueue<Order>> _orderQueue;
};

#endif