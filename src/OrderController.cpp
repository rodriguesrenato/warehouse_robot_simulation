#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <deque>
#include <condition_variable>
#include <chrono>
#include "WarehouseObject.h"

#include "OrderController.h"
#include "Order.h"

OrderController::OrderController(std::string orderControllerName)
{
    _orderControllerName = orderControllerName + "#" + std::to_string(_id);
    _objectName = orderControllerName + "#" + std::to_string(_id);
}
OrderController::~OrderController()
{
    Print("Destructor");
}

void OrderController::AddOrder(const std_msgs::String &str)
{
    std::shared_ptr<Order> order = std::make_shared<Order>("order");
    try
    {
        std::istringstream stream(str.data);
        std::string val;
        if (stream)
        {
            stream >> val;
            order->SetGoalDispatchName(val);
            while (!stream.eof())
            {
                std::string product;
                std::string productQuantity;

                stream >> product >> productQuantity;

                // TODO: validate order values, discard not valid prod and quant
                order->AddProduct(product, stoi(productQuantity));
            }
        }
    }
    catch (...)
    {
        Print("Could not process this Order request: " + str.data);
        return;
    }

    if (order->GetProductList().bucket_count() > 0)
    {
        std::lock_guard<std::mutex> lck(_queueMtx);
        _queue.push_back(std::move(order));
        _queueCond.notify_one();

        Print(_queue.back()->GetName() + " was added to queue[" + std::to_string(_queue.size()) + "]");
    }
}

std::shared_ptr<Order> OrderController::RequestNextOrder(std::string robotName) // TODO: Usar aqui as condition variables para que os robos facam as requests e aguardem um order chegar
{
    Print(robotName + " is requesting an Order");
    std::unique_lock<std::mutex> uLck(_queueMtx);
    _queueCond.wait(uLck, [this] { return !_queue.empty(); });

    std::shared_ptr<Order> order = std::move(_queue.front());
    _queue.pop_front();

    order->SetRobotWorkerName(robotName);
    _orders.push_back(order);

    Print(order->GetName() + " given to " + robotName);

    return order;
}

std::shared_ptr<Order> OrderController::RequestNextOrderWithTimeout(std::string robotName, int timeoutMs) // TODO: Usar aqui as condition variables para que os robos facam as requests e aguardem um order chegar
{
    // Print(robotName + " is requesting an Order with timeout of " + std::to_string(timeoutMs) + "ms");
    
    std::unique_lock<std::mutex> uLck(_queueMtx);
    auto cv = _queueCond.wait_for(uLck, std::chrono::milliseconds(timeoutMs));

    // if timeout, return a nullptr
    if (cv == std::cv_status::timeout)
    {
        return nullptr;
    }

    std::shared_ptr<Order> order = std::move(_queue.front());
    _queue.pop_front();

    order->SetRobotWorkerName(robotName);
    _orders.push_back(order);

    Print(order->GetName() + " was given to " + robotName);

    return order;
}
void OrderController::CloseOrder(std::shared_ptr<Order> order)
{
    // TODO
    std::vector<std::shared_ptr<Order>>::iterator pos = std::find(_orders.begin(), _orders.end(), order);
    if (pos != _orders.end())
        _orders.erase(pos);
}