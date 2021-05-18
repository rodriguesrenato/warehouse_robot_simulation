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

#include "Order.h"
#include "Robot.h"
#include "OrderController.h"
#include "Storage.h"
#include "Product.h"
#include "Model.h"
#include "WarehouseObject.h"

Robot::Robot(std::string robotName, std::vector<std::shared_ptr<Storage>> &storages, std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<OrderController> orderController)
{
    _robotName = robotName + "#" + std::to_string(_id);
    _type = ObjectType::objectRobot;
    _storages = storages;
    _dispatches = dispatches;
    _orderController = orderController;
    // std::cout << "##### _storage size is " << _storages.size() << std::endl;
}
Robot::~Robot()
{
    _status = StatusType::standby;
}

void Robot::SetStatus(StatusType status)
{
    _status = status;
}
StatusType Robot::GetStatus()
{
    return _status;
}

void Robot::SetOrder(Order order)
{
}
void Robot::SetWarehouseObjects(std::vector<std::shared_ptr<Storage>> storages, std::vector<std::shared_ptr<Dispatch>> dispatches)
{
    _storages = storages;
    _dispatches = dispatches;
}
bool Robot::isStandby()
{
}
std::deque<std::shared_ptr<Storage>> Robot::GetStoragesToGo()
{
    std::unordered_map<std::string, int> productList = _order->GetProductList();
    // std::unordered_map<std::string, int> storageList;
    std::deque<std::shared_ptr<Storage>> storagesToGo;

    for (int i = 0; i < _storages.size(); i++)
    {

        //Check if there is a storage that produces a product in the productList;
        if (productList.find(_storages[i]->GetProductionModelName()) != productList.end())
        {
            // Add this storage to storagesToGo vector and remove the product from productList to avoid repeting
            storagesToGo.push_back(_storages[i]);
            productList.erase(_storages[i]->GetProductionModelName());
            // add to storageList the quantity of product needed
            // storageList[_storages[i]->GetName()] = productList[_storages[i]->GetProductionModelName()];
        }
    };
    return storagesToGo;
}

void Robot::StartOperation()
{
    _status = StatusType::requestOrder;
    threads.emplace_back(std::thread(&Robot::ExecuteOrder, this));
}

void Robot::ExecuteOrder()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "[" << _robotName << "] >> ExecuteOrder" << std::endl;
    while (_status != StatusType::standby)
    {
        _order = _orderController->RequestNextOrder(_robotName); // TODO add mutex
        std::cout << "[" << _robotName << "] Receive order " << _order->GetOrderName() << " - Find next storage to go!" << std::endl;

        std::deque<std::shared_ptr<Storage>> storagesToGo = this->GetStoragesToGo();

        std::shared_ptr<Dispatch> dispatchToGo;
        for (std::shared_ptr<Dispatch> &d : _dispatches)
        {
            std::cout << "[" << d->GetModelName() << "] Dispatch " << _order->GetGoalDispatchName() << std::endl;
            if (d->GetModelName() == _order->GetGoalDispatchName()){
                dispatchToGo = d;
            }
        };
        std::cout << "[" << _robotName << "] Dispatch " << dispatchToGo->GetName() << std::endl;
        std::cout << "[" << _robotName << "] Dispatch pos_x " << dispatchToGo->getPose().position.x << std::endl;
        // Navigate to the storages locations and request products
        while (storagesToGo.size() > 0)
        {
            std::shared_ptr<Storage> targetStorage = storagesToGo.front();
            storagesToGo.pop_front();
            std::cout << "[" << _robotName << "] Moving to " << targetStorage->GetName() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "[" << _robotName << "] Arrived at " << targetStorage->GetName() << " | Position (" << targetStorage->getPose().position.x << "," << targetStorage->getPose().position.y << "," << targetStorage->getPose().position.z << ")" << std::endl;

            std::cout << "[" << _robotName << "] Request Products" << std::endl;
            int productQuantity = _order->GetProductList()[targetStorage->GetProductionModelName()];
            for (int i = 0; i < productQuantity; i++)
            {
                std::unique_ptr<Product> p = targetStorage->RequestProduct();
                if (p != nullptr)
                {
                    std::cout << "[" << _robotName << "] take product " << p->GetName() << std::endl;
                    _takenProducts.push_back(std::move(p));
                }
                else
                {
                    i--;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            std::cout << "[" << _robotName << "] take all products from " << targetStorage->GetName() << std::endl;
        }
        std::cout << "[" << _robotName << "] Moving to " << _order->GetGoalDispatchName() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout << "[" << _robotName << "] Arrived to "<< _order->GetGoalDispatchName() << " | Position (" << dispatchToGo->getPose().position.x << "," << dispatchToGo->getPose().position.y << "," << dispatchToGo->getPose().position.z << ")" << std::endl;
        
        for (std::unique_ptr<Product> &p : _takenProducts)
        {
            std::cout << "[" << _robotName << "] Dispatch is picking" << p->GetName() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            dispatchToGo->PickProduct(std::move(p));
        }
        // std::cout << "[" << _robotName << "] Moving to " < < < < std::endl;
        std::cout << _order->GetOrderName() << " is completed!" << std::endl;
    }
}