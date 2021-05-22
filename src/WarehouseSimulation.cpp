#include "ros/ros.h"

#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ios>
#include <math.h>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <future>

#include "Model.h"
#include "Storage.h"
#include "Dispatch.h"
#include "Product.h"
#include "Order.h"
#include "OrderController.h"
#include "Robot.h"
#include <signal.h>

bool isShutdown = false;

void loadModels(std::shared_ptr<Model> &modelController)
{
    modelController->Add("productA", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/productA.sdf");
    modelController->Add("productB", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/productB.sdf");
    modelController->Add("storageA", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/storageA.sdf");
    modelController->Add("storageB", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/storageA.sdf");
    modelController->Add("dispatchA", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/dispatchA.sdf");
    modelController->Add("dispatchB", "/home/renato/catkin_ws/src/delivery_robot_simulation/models/dispatchA.sdf");
}

void InstatiateWarehouseObjects(std::vector<std::shared_ptr<Storage>> &storages, std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<Model> modelController, std::string configsDirectory)
{
    std::string line;

    std::ifstream storagesConfigFs(configsDirectory + "storages");
    if (storagesConfigFs.is_open())
    {
        while (std::getline(storagesConfigFs, line))
        {
            if (line[0] == '#')
            {
                continue;
            }
            std::istringstream linestream(line);
            std::string storageModel{};
            std::string productionModel{};
            geometry_msgs::Pose storagePose{};
            geometry_msgs::Pose productOutputPose{};
            std::string psx{};
            std::string psy{};
            std::string psz{};
            std::string ppx{};
            std::string ppy{};
            std::string ppz{};

            linestream >> storageModel;

            linestream >> productionModel;

            linestream >> psx >> psy >> psz;
            storagePose.position.x = std::stof(psx);
            storagePose.position.y = std::stof(psy);
            storagePose.position.z = std::stof(psz);

            linestream >> ppx >> ppy >> ppz;
            productOutputPose.position.x = std::stof(ppx);
            productOutputPose.position.y = std::stof(ppy);
            productOutputPose.position.z = std::stof(ppz);

            storages.push_back(std::make_shared<Storage>(storageModel, productionModel, storagePose, productOutputPose, modelController));
        }
    }
    storagesConfigFs.close();

    std::ifstream dispatchesConfigFs(configsDirectory + "dispatches");
    if (dispatchesConfigFs.is_open())
    {
        while (std::getline(dispatchesConfigFs, line))
        {
            if (line[0] == '#')
            {
                continue;
            }
            std::istringstream linestream(line);
            std::string dispatchModel{};
            geometry_msgs::Pose dispatchPose{};
            std::string psx{};
            std::string psy{};
            std::string psz{};

            linestream >> dispatchModel;

            linestream >> psx >> psy >> psz;
            dispatchPose.position.x = std::stof(psx);
            dispatchPose.position.y = std::stof(psy);
            dispatchPose.position.z = std::stof(psz);

            dispatches.push_back(std::make_shared<Dispatch>(dispatchModel, dispatchPose, modelController));
        }
    }
    dispatchesConfigFs.close();
}

void mySigintHandler(int sig)
{
    std::cout << "Simulation is being shut down" << std::endl;
    isShutdown = true;
    // ros::shutdown();
}

int main(int argc, char **argv)
{
    // Initialize the WarehouseSimulation node and create a handle to it
    ros::init(argc, argv, "WarehouseSimulation", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    // Create an instance of the modelController
    std::shared_ptr<Model> modelController = std::make_shared<Model>();

    // Create and instance of the orderController
    std::shared_ptr<OrderController> orderController = std::make_shared<OrderController>("warehouseOrderController");

    // Create Storage objects containers
    std::vector<std::shared_ptr<Storage>> storages;
    std::vector<std::shared_ptr<Dispatch>> dispatches;
    std::vector<std::shared_ptr<Robot>> robots;

    // Load models
    loadModels(modelController);

    // Instantiate objects from config file
    InstatiateWarehouseObjects(storages, dispatches, modelController, "/home/renato/catkin_ws/src/delivery_robot_simulation/configs/");

    // Create a Robot Object and configure it
    robots.emplace_back(std::make_shared<Robot>("amr", "move_base", storages, dispatches, orderController));

    // Add Storage model to simulation and start it
    std::for_each(storages.begin(), storages.end(), [modelController](std::shared_ptr<Storage> &s) {
        modelController->Spawn(s->GetName(), s->GetModelName(), s->GetPose());
        s->StartOperation();
    });

    std::for_each(dispatches.begin(), dispatches.end(), [modelController](std::shared_ptr<Dispatch> &d) {
        modelController->Spawn(d->GetName(), d->GetModelName(), d->GetPose());
    });

    std::for_each(robots.begin(), robots.end(), [modelController](std::shared_ptr<Robot> &r) {
        r->StartOperation();
    });

    ros::Subscriber ordersSubscriber = n.subscribe("warehouse/order/add", 10, &OrderController::AddOrder, &*orderController);

    // Set a sigHandler to handle CRTL+C
    signal(SIGINT, mySigintHandler);

    ros::Rate loop_rate(10);
    while (ros::ok() && !isShutdown)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::for_each(storages.begin(), storages.end(), [modelController](std::shared_ptr<Storage> &s) {
        modelController->Delete(s->GetName());
    });

    std::for_each(dispatches.begin(), dispatches.end(), [modelController](std::shared_ptr<Dispatch> &d) {
        modelController->Delete(d->GetName());
    });

    std::for_each(robots.begin(), robots.end(), [modelController](std::shared_ptr<Robot> &r) {
        for(auto &p:r->GetTakenProducts()){
        modelController->Delete(p->GetName());
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // After Delete spawned models in Gazebo, then completely shutdown this ros node
    ros::shutdown();
    return 0;
}

/*
Custom SIGINT Handler
You can install a custom SIGINT handler that plays nice with ROS like so:

Toggle line numbers
   1 #include <ros/ros.h>
   2 #include <signal.h>
   3 
   4 void mySigintHandler(int sig)
   5 {
   6   // Do some custom action.
   7   // For example, publish a stop message to some other nodes.
   8   
   9   // All the default sigint handler does is call shutdown()
  10   ros::shutdown();
  11 }
  12 
  13 int main(int argc, char** argv)
  14 {
  15   ros::init(argc, argv, "my_node_name", ros::init_options::NoSigintHandler);
  16   ros::NodeHandle nh;
  17 
  18   // Override the default ros sigint handler.
  19   // This must be set after the first NodeHandle is created.
  20   signal(SIGINT, mySigintHandler);
  21   
  22   //...
  23   ros::spin();
  24   return 0;
  25 }

*/

// std::for_each(futures.begin(), futures.end(), [](std::future<std::shared_ptr<Order>> &ftr) {
//     ftr.wait();
// });