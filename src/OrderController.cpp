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
}
OrderController::~OrderController()
{
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
            // std::cout << "goalDispatchName: " << val << std::endl;
            order->SetGoalDispatchName(val);
            while (!stream.eof())
            {
                std::string product;
                std::string productQuantity;

                stream >> product >> productQuantity;
                // std::cout << "product: " << product << std::endl;
                // std::cout << "productQuantity: " << productQuantity << std::endl;
                // TODO: validate order values, discard not valid prod and quant
                order->AddProduct(product, stoi(productQuantity));
            }
        }
    }
    catch (...)
    {
        std::cout << "Could not process this Order request: " << str << std::endl;
        return;
    }

    if (order->GetProductList().bucket_count() > 0)
    {
        std::lock_guard<std::mutex> lck(_queueMtx);
        _queue.push_back(std::move(order));
        _queueCond.notify_one();

        std::cout << _queue.back()->GetOrderName() << " was added to queue[" << _queue.size() << "]" << std::endl;
    }
}

std::shared_ptr<Order> OrderController::RequestNextOrder(std::string robotName) // TODO: Usar aqui as condition variables para que os robos facam as requests e aguardem um order chegar
{
    std::unique_lock<std::mutex> uLck(_queueMtx);
    std::cout << robotName << " is requesting an Order " << std::endl;
    _queueCond.wait(uLck, [this] { return !_queue.empty(); });

    std::shared_ptr<Order> order = std::move(_queue.front());
    _queue.pop_front();

    order->SetRobotWorkerName(robotName);
    _orders.push_back(order);

    std::cout << order->GetOrderName() << " given to " << robotName << std::endl;

    return order;
}

std::shared_ptr<Order> OrderController::RequestNextOrderWithTimeout(std::string robotName, int timeout) // TODO: Usar aqui as condition variables para que os robos facam as requests e aguardem um order chegar
{
    std::unique_lock<std::mutex> uLck(_queueMtx);
    std::cout << robotName << " is requesting an Order with timeout of " << timeout << "ms" << std::endl;
    auto cv = _queueCond.wait_for(uLck, std::chrono::milliseconds(timeout));

    // if timeout, return a nullptr
    if (cv == std::cv_status::timeout)
    {
        return nullptr;
    }

    std::shared_ptr<Order> order = std::move(_queue.front());
    _queue.pop_front();

    order->SetRobotWorkerName(robotName);
    _orders.push_back(order);

    std::cout << order->GetOrderName() << " given to " << robotName << std::endl;

    return order;
}
void OrderController::CloseOrder(std::shared_ptr<Order> order)
{
    // TODO
    std::vector<std::shared_ptr<Order>>::iterator pos = std::find(_orders.begin(), _orders.end(), order);
    if (pos != _orders.end())
        _orders.erase(pos);
}