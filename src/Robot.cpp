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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>

#include "Order.h"
#include "Robot.h"
#include "OrderController.h"
#include "Storage.h"
#include "Product.h"
#include "Model.h"
#include "WarehouseObject.h"

Robot::Robot(std::string robotName, std::string actionName, std::vector<std::shared_ptr<Storage>> &storages, std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<OrderController> orderController)
{
    _robotName = robotName + "#" + std::to_string(_id);
    _objectName = robotName + "#" + std::to_string(_id);
    ;
    _type = ObjectType::objectRobot;
    _storages = storages;
    _dispatches = dispatches;
    _orderController = orderController;
    _actionName = actionName;
    // std::cout << "##### _storage size is " << _storages.size() << std::endl;
}
Robot::~Robot()
{
    Print("Destructor");
    _status = StatusType::offline;
}

void Robot::SetStatus(StatusType status)
{
    _status = status;
}
StatusType Robot::GetStatus()
{
    return _status;
}

std::vector<std::unique_ptr<Product>> &Robot::GetTakenProducts()
{
    return _takenProducts;
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
    return _status == StatusType::standby;
}
std::deque<std::shared_ptr<Storage>> Robot::GetStoragesToGo()
{
    std::unordered_map<std::string, int> productList = _order->GetProductList();
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
    _status = StatusType::startup;
    threads.emplace_back(std::thread(&Robot::ExecuteOrder, this));
}

bool Robot::Move(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac, geometry_msgs::Pose goal)
{
    // Convert Pose orientation to quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(goal.orientation.x, goal.orientation.y, goal.orientation.z);

    // define a MoveBaseGoal from goal
    move_base_msgs::MoveBaseGoal mbGoal;
    mbGoal.target_pose.header.frame_id = "map";
    mbGoal.target_pose.header.stamp = ros::Time::now();
    mbGoal.target_pose.pose.position.x = goal.position.x;
    mbGoal.target_pose.pose.position.y = goal.position.y;
    mbGoal.target_pose.pose.position.z = goal.position.z;
    mbGoal.target_pose.pose.orientation.x = quaternion.x();
    mbGoal.target_pose.pose.orientation.y = quaternion.y();
    mbGoal.target_pose.pose.orientation.z = quaternion.z();
    mbGoal.target_pose.pose.orientation.w = quaternion.w();

    ac.sendGoal(mbGoal);

    ac.waitForResult();

    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void Robot::ExecuteOrder()
{
    Print(" ExecuteOrder thread started");

    // Create the Action Client and wait for it's initialisation
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(_actionName, true);
    std::shared_ptr<Dispatch> targetDispatch;
    std::deque<std::shared_ptr<Storage>> storagesToGo;
    std::shared_ptr<Storage> targetStorage;

    while (_status != StatusType::offline)
    {
        switch (_status)
        {
        case StatusType::startup:
            if (ac.waitForServer(ros::Duration(5.0)))
            {
                Print("ActionServer connected");
                _status = StatusType::requestOrder;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;

        case StatusType::standby:

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break;

        case StatusType::requestOrder:

            _order = _orderController->RequestNextOrderWithTimeout(_robotName, 1000);

            if (_order != nullptr)
            {
                Print("Received order " + _order->GetName());

                _status = StatusType::processOrder;
            }
            break;

        case StatusType::processOrder:

            //clear previous values
            targetStorage.reset();
            targetDispatch.reset();
            storagesToGo.clear();

            // Get a queue of storages that has order's products
            storagesToGo = this->GetStoragesToGo();

            // Get the dispatch shared_ptr by dispatch modelName
            for (std::shared_ptr<Dispatch> &d : _dispatches)
            {
                if (d->GetModelName() == _order->GetGoalDispatchName())
                {
                    targetDispatch = d;
                }
            }
            _status = StatusType::plan;
            break;

        case StatusType::plan:

            // Move to the storages one by one to get the products
            if (storagesToGo.size() > 0)
            {
                // Get first storage and remove it from queue
                targetStorage = storagesToGo.front();
                storagesToGo.pop_front();
                _status = StatusType::moveToStorage;
            }
            else
            {
                if (targetDispatch != nullptr)
                {
                    _status = StatusType::moveToDispatch;
                }
                else
                {
                    _status = StatusType::requestOrder;
                }
            }
            break;

        case StatusType::moveToStorage:
            // Move to this storage, if fails, try until get there
            if (targetStorage != nullptr)
            {
                Print(" Moving to " + targetStorage->GetName());
                if (Move(ac, targetStorage->GetProductOutputPose()))
                {
                    Print("Arrive at " + targetStorage->GetName() + ", requesting products");
                    _status = StatusType::executeOrder;
                }
                else
                {
                    Print("Fail to move to " + targetStorage->GetName() + ", trying again..");
                }
            }
            else
            {
                // TODO
            }
            break;

        case StatusType::executeOrder:
            // Get the number of products placed in order
            // int productQuantity = _order->GetProductList()[targetStorage->GetProductionModelName()];

            for (int i = 0; i < _order->GetProductList()[targetStorage->GetProductionModelName()]; i++)
            {
                // check if _status haven't changed
                if (_status != StatusType::executeOrder)
                {
                    break;
                }

                // Request one product at time
                std::unique_ptr<Product> p = targetStorage->RequestProduct();

                if (p != nullptr)
                {
                    // If receive a product, store it's reference in _takenProducts
                    Print("Got " + p->GetName());
                    _takenProducts.push_back(std::move(p));
                }
                else
                {
                    // If storage is empty or received an invalid product, wait a bit more for storage produce more products
                    i--;
                }

                // Wait for product accomodate in the robot's cargo bed before request a new product
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            Print(std::to_string(_takenProducts.size()) + " " + targetStorage->GetProductionModelName() + " were gotten at " + targetStorage->GetName());
            _status = StatusType::plan;
            break;

        case StatusType::moveToDispatch:
            // After get all products in the order, move to the target Dispatch area

            Print(" Moving to " + targetDispatch->GetName());
            if (Move(ac, targetDispatch->GetPose()))
            {
                Print("Arrive at " + targetDispatch->GetName() + ", dispatch" + std::to_string(_takenProducts.size()) + " products");
                _status = StatusType::dispatchOrder;
            }
            else
            {
                Print("Fail to move to " + targetDispatch->GetName() + ", trying again..");
            }

            break;

        case StatusType::dispatchOrder:
            // Deliver all products that were taken
            while (_takenProducts.size() > 0)
            {
                Print("Deliver " + _takenProducts.back()->GetName() + " to " + targetDispatch->GetName());

                // Simulate a small delay between deliveries and move Product from robot cargo bed to Dispatch
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                targetDispatch->PickProduct(std::move(_takenProducts.back()));
                _takenProducts.pop_back();
            }

            Print(_order->GetName() + " is completed!");
            _status = StatusType::requestOrder;
            break;
        }
    }
}