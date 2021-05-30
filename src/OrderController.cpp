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

// Define an unique OrderController name
OrderController::OrderController(std::string orderControllerName)
{
    _objectName = orderControllerName + "#" + std::to_string(_id);
}

OrderController::~OrderController()
{
    Print("Shutting down");
}

// Process an Order received from ros and add it to _queue
void OrderController::AddOrder(const std_msgs::String &str)
{
    // Create and empty Order
    std::shared_ptr<Order> order = std::make_shared<Order>("RosOrder");

    // Parse the order received in plain string format:
    // TargetDispatchName Product1Name Product1Quantity Product2Name Product2Quantity ...
    try
    {
        std::istringstream stream(str.data);
        std::string val;
        if (stream)
        {
            stream >> val;

            // Set first val as TargetDispatchName
            order->SetGoalDispatchName(val);

            // Then parse all following listed Products
            while (!stream.eof())
            {
                std::string product;
                std::string productQuantityStr;
                stream >> product >> productQuantityStr;

                int productQuantity = stoi(productQuantityStr);

                // If quantity is greater than zero, then add this product to the order
                if (productQuantity > 0)
                {
                    order->AddProduct(product, productQuantity);
                }
            }
        }
    }
    // If any parsing error occurred, then discard this order request
    catch (...)
    {
        Print("Could not process this Order request: " + str.data);
        return;
    }

    // If the parsed order has more than one product, then add it to the end of queue
    if (order->GetProductList().bucket_count() > 0)
    {
        std::lock_guard<std::mutex> lck(_queueMtx);
        _queue.push_back(std::move(order));
        _queueCond.notify_one();

        Print(_queue.back()->GetName() + "|" + _queue.back()->GetGoalDispatchName() + " was added to queue[" + std::to_string(_queue.size()) + "]");
    }
}

// Remove the order from _ordersTracking
void OrderController::CloseOrder(std::shared_ptr<Order> order)
{
    // Find the Order position in _ordersTracking vector and erase it
    std::lock_guard<std::mutex> uTLck(_ordersTrackingMtx);
    std::vector<std::shared_ptr<Order>>::iterator pos = std::find(_ordersTracking.begin(), _ordersTracking.end(), order);
    if (pos != _ordersTracking.end())
    {
        _ordersTracking.erase(pos);
    }
}

// Return a vector of orders that are being tracked
std::vector<std::shared_ptr<Order>> OrderController::GetOrdersTracking()
{
    std::lock_guard<std::mutex> uTLck(_ordersTrackingMtx);
    return _ordersTracking;
}

// Return the next Order in the queue
std::shared_ptr<Order> OrderController::RequestNextOrder(std::string robotName)
{
    Print(robotName + " is requesting an Order");

    // Use a condition variable to wait until the queue is not empty
    std::unique_lock<std::mutex> uLck(_queueMtx);
    _queueCond.wait(uLck, [this]
                    { return !_queue.empty(); });

    // Get the first Order from queue and release the lock
    std::shared_ptr<Order> order = std::move(_queue.front());
    _queue.pop_front();
    uLck.unlock();

    // Set the robotName that is requesting the order
    order->SetRobotWorkerName(robotName);
    std::unique_lock<std::mutex> uTLck(_ordersTrackingMtx);
    _ordersTracking.push_back(order);
    uTLck.unlock();

    Print(order->GetName() + " is given to " + robotName);

    return order;
}

// Return the next Order in the queue with a timeout option that returns a nullptr if _queue still empty
std::shared_ptr<Order> OrderController::RequestNextOrderWithTimeout(std::string robotName, int timeoutMs) // TODO: Usar aqui as condition variables para que os robos facam as requests e aguardem um order chegar
{
    // Use a condition variable to wait until the queue is not empty
    std::unique_lock<std::mutex> uLck(_queueMtx);
    auto cv = _queueCond.wait_for(uLck, std::chrono::milliseconds(timeoutMs));

    // If order queue is empty, then return a nullptr
    if (_queue.empty())
    {
        return nullptr;
    }

    // Get the first Order from queue and release the lock
    std::shared_ptr<Order> order = std::move(_queue.front());
    _queue.pop_front();
    uLck.unlock();

    // Set the robotName that is requesting the order
    order->SetRobotWorkerName(robotName);
    std::unique_lock<std::mutex> uTLck(_ordersTrackingMtx);
    _ordersTracking.push_back(order);
    uTLck.unlock();

    Print(order->GetName() + " was given to " + robotName);

    return order;
}