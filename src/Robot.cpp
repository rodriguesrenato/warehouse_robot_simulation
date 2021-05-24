#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <chrono>

#include <geometry_msgs/Pose.h>
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

// Define a unique robot name, set the respective move_base action serve name and others warehouse objects shared pointers
Robot::Robot(std::string robotName, std::string actionName, std::vector<std::shared_ptr<Storage>> &storages, std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<OrderController> orderController)
{
    _objectName = robotName + "#" + std::to_string(_id);
    _type = ObjectType::objectRobot;
    _storages = storages;
    _dispatches = dispatches;
    _orderController = orderController;
    _actionName = actionName;
}

// On Destructor, set _status to offline to finish internally any running thread
Robot::~Robot()
{
    Print("Shutting down");
    _status = RobotStatus::offline;
}

// Set the robot _status
void Robot::SetStatus(RobotStatus status)
{
    _status = status;
}

// Return current robot _status
RobotStatus Robot::GetStatus()
{
    return _status;
}

// Return a vector of products names that are on robot cargo bin
std::vector<std::string> Robot::GetCargoBinProductsName()
{
    std::lock_guard<std::mutex> lck(_cargoBinMtx);
    std::vector<std::string> productsNames;

    for (auto &p : _cargoBinProducts)
    {
        productsNames.push_back(p->GetName());
    }

    return productsNames;
}

// Start the robot Operate thread
void Robot::StartOperation()
{
    // Only start the Operate thread if robot status is offline
    if (_status == RobotStatus::offline)
    {
        _status = RobotStatus::startup;
        threads.emplace_back(std::thread(&Robot::Operate, this));
    }
}

// Return a queue of Storages that contains the products listed in the current order
std::deque<std::shared_ptr<Storage>> Robot::GetStoragesToGo()
{
    std::unordered_map<std::string, int> productList = _order->GetProductList();
    std::deque<std::shared_ptr<Storage>> storagesToGo;

    // Interate over all storages available in the warehouse
    // Check if there is a storage that produces a product in the productList;
    for (int i = 0; i < _storages.size(); i++)
    {
        // Check if the product that the storage produces is on the productList
        if (productList.find(_storages[i]->GetProductionModelName()) != productList.end())
        {
            // Add this storage to storagesToGo vector and remove the product from productList to avoid repeting
            storagesToGo.push_back(_storages[i]);
            productList.erase(_storages[i]->GetProductionModelName());
        }
    };
    return storagesToGo;
}

// Move Robot to the desired goal Pose
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

    // Send the MoveBaseGoal to the SimpleActionClient and wait for thr result
    ac.sendGoal(mbGoal);
    ac.waitForResult();

    // Return true if robot succeeded to reach the desired goal, otherwise false
    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

// Make robot operating accordingly to it's RobotStatuss, until robot _status is offline
void Robot::Operate()
{
    Print("Started Operating");

    // Create the Action Client and current operational shared variables
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(_actionName, true);
    std::shared_ptr<Dispatch> targetDispatch;
    std::deque<std::shared_ptr<Storage>> storagesToGo;
    std::shared_ptr<Storage> targetStorage;

    // Iterate over _status until it get set to offline
    while (_status != RobotStatus::offline)
    {
        switch (_status)
        {
        // Initialize the SimpleActionClient and wait until it gets ready to change the _status
        case RobotStatus::startup:

            if (ac.waitForServer(ros::Duration(5.0)))
            {
                Print("Startup complete, SimpleActionClient connected");
                _status = RobotStatus::requestOrder;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;

        // Wait for an external _status set
        case RobotStatus::standby:

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break;

        // Request OrderController for an Order
        case RobotStatus::requestOrder:

            _order = _orderController->RequestNextOrderWithTimeout(_objectName, 1000);

            if (_order != nullptr)
            {
                Print("Received order " + _order->GetName());

                _status = RobotStatus::processOrder;
            }
            break;

        // Clear the operational variables and proper set them
        case RobotStatus::processOrder:

            //clear previous values
            targetStorage.reset();
            targetDispatch.reset();
            storagesToGo.clear();

            // Get a queue of storages that has order's products
            storagesToGo = this->GetStoragesToGo();

            // Set targetDispatch by dispatch modelName
            for (std::shared_ptr<Dispatch> &d : _dispatches)
            {
                if (d->GetModelName() == _order->GetGoalDispatchName())
                {
                    targetDispatch = d;
                }
            }
            _status = RobotStatus::plan;
            break;

        // Plan and set the next robot task
        case RobotStatus::plan:

            // Move to the storages one by one to get the products
            if (storagesToGo.size() > 0)
            {
                // Get first storage and remove it from queue
                targetStorage = storagesToGo.front();
                storagesToGo.pop_front();

                if (targetStorage != nullptr)
                {
                    _status = RobotStatus::moveToStorage;
                }
            }
            // When all storages were visited, then go to the dispatch area
            else
            {
                if (targetDispatch != nullptr)
                {
                    _status = RobotStatus::moveToDispatch;
                }
                else
                {
                    // In case of operational variable set as nullptr, request a new order
                    _status = RobotStatus::requestOrder;
                }
            }
            break;

        // Move robot to the targetStorage
        case RobotStatus::moveToStorage:

            Print(" Moving to " + targetStorage->GetName());
            if (Move(ac, targetStorage->GetProductOutputPose()))
            {
                Print("Arrive at " + targetStorage->GetName() + ", requesting products");
                _status = RobotStatus::requestProduct;
            }

            // If robot fails to move to the targetStorage, _status will be the same and robot will try again in the next iteration
            else
            {
                Print("Fail to move to " + targetStorage->GetName() + ", trying again..");
            }

            break;

        // Request the products in the targetStorage
        case RobotStatus::requestProduct:

            // Iterate until the quantity of this product in the order were gotten
            for (int i = 0; i < _order->GetProductList()[targetStorage->GetProductionModelName()]; i++)
            {
                // check if _status haven't changed
                if (_status != RobotStatus::requestProduct)
                {
                    break;
                }

                // Request one product at time
                std::unique_ptr<Product> p = targetStorage->RequestProduct();

                // If it received a product, move this product to _cargoBinProducts
                if (p != nullptr)
                {
                    Print("Got " + p->GetName());
                    std::lock_guard<std::mutex> lck(_cargoBinMtx);
                    _cargoBinProducts.push_back(std::move(p));
                }
                // If storage is empty or received an invalid product, wait a bit more for storage produce more products
                else
                {
                    i--;
                }

                // Wait for product accomodate in the robot's cargo bed before request a new product and make this for loop safe of unnecessary processing when no product is received
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            Print(std::to_string(_cargoBinProducts.size()) + " " + targetStorage->GetProductionModelName() + " were gotten at " + targetStorage->GetName());

            // return to plan status to define next task
            _status = RobotStatus::plan;
            break;

        // After get all products in the order, move to the target Dispatch area
        case RobotStatus::moveToDispatch:

            Print(" Moving to " + targetDispatch->GetName());
            if (Move(ac, targetDispatch->GetPose()))
            {
                Print("Arrive at " + targetDispatch->GetName() + ", dispatch" + std::to_string(_cargoBinProducts.size()) + " products");
                _status = RobotStatus::dispatchOrder;
            }
            // If robot fails to move to the targetDispatch, _status will be the same and robot will try again in the next iteration
            else
            {
                Print("Fail to move to " + targetDispatch->GetName() + ", trying again..");
            }

            break;

        // Deliver all products in the _cargoBinProducts
        case RobotStatus::dispatchOrder:

            // Deliver all products that were taken
            while (_cargoBinProducts.size() > 0)
            {
                Print("Delivering " + _cargoBinProducts.back()->GetName() + " to " + targetDispatch->GetName());

                // Move Product to the targetDispach and remove it from _cargoBinProducts
                std::unique_lock<std::mutex> lck(_cargoBinMtx);
                targetDispatch->PickProduct(std::move(_cargoBinProducts.back()));
                _cargoBinProducts.pop_back();
                lck.unlock();

                // Simulate a small delay between deliveries and move Product from robot cargo bed to Dispatch
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            Print(_order->GetName() + " is completed!");
            _order.reset();

            // After the current order gets completed, request a new one
            _status = RobotStatus::requestOrder;
            break;
        }
    }
}