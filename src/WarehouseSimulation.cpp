#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ios>
#include <math.h>
#include <memory>
#include <thread>
#include <exception>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf2/LinearMath/Quaternion.h>

#include "ModelController.h"
#include "Storage.h"
#include "Dispatch.h"
#include "Product.h"
#include "Order.h"
#include "OrderController.h"
#include "Robot.h"
#include <signal.h>

bool isShutdown = false;

// Print function to make code cleaner and easier to track on cout
void Print(std::string message)
{
    std::cout << "[WarehouseSimulation] " << message << std::endl;
}

// Custom SIGINT handler
void SigintHandler(int sig)
{
    std::cout << "Simulation is being shut down" << std::endl;

    // Set isShutdown true to break ros spin while loop and start simulation cleaning spawned object
    isShutdown = true;
}

// Build and return a geometry_msgs::Pose from string values of xyzrpy
geometry_msgs::Pose GetPoseFromString(float x, float y, float z, float roll, float pitch, float yaw)
{
    geometry_msgs::Pose pose;
    tf2::Quaternion quaternionPose;

    // Set x, y and z values
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Convert RPY orientation to quaternion
    quaternionPose.setRPY(roll, pitch, yaw);
    pose.orientation.x = quaternionPose.x();
    pose.orientation.y = quaternionPose.y();
    pose.orientation.z = quaternionPose.z();
    pose.orientation.w = quaternionPose.w();

    return pose;
}

// Load all specified models to the model controller
void LoadModels(std::shared_ptr<ModelController> &modelController, std::string projectDirectory)
{
    modelController->Add("ProductR", projectDirectory + "/models/warehouseObjects/ProductR.sdf");
    modelController->Add("ProductG", projectDirectory + "/models/warehouseObjects/ProductG.sdf");
    modelController->Add("ProductB", projectDirectory + "/models/warehouseObjects/ProductB.sdf");

    modelController->Add("StorageR", projectDirectory + "/models/warehouseObjects/StorageR.sdf");
    modelController->Add("StorageG", projectDirectory + "/models/warehouseObjects/StorageG.sdf");
    modelController->Add("StorageB", projectDirectory + "/models/warehouseObjects/StorageB.sdf");

    modelController->Add("DispatchA", projectDirectory + "/models/warehouseObjects/Dispatch.sdf");
    modelController->Add("DispatchB", projectDirectory + "/models/warehouseObjects/Dispatch.sdf");
}

// Read the configuration files and instantiate the warehouse objects
bool InstatiateWarehouseObjects(std::vector<std::shared_ptr<Storage>> &storages, std::vector<std::shared_ptr<Dispatch>> &dispatches, std::shared_ptr<ModelController> modelController, std::string projectDirectory)
{
    std::string line;

    // Add Storages from storages configuration file
    try
    {
        // Read Storages configuration file
        std::ifstream storagesConfigFs(projectDirectory + "/configs/storages");
        if (storagesConfigFs.is_open())
        {
            while (std::getline(storagesConfigFs, line))
            {
                // if line started with #, then discard this line
                // # is used as user configuration description
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
                std::string osx{};
                std::string osy{};
                std::string osz{};
                std::string ppx{};
                std::string ppy{};
                std::string ppz{};
                std::string opx{};
                std::string opy{};
                std::string opz{};
                std::string maxCapacity{};

                // Get the Storage model name
                linestream >> storageModel;

                // Get the Storage's Product name that will be used in Production
                linestream >> productionModel;

                // Get the Storage's max capacity of stored Product
                linestream >> maxCapacity;

                // Get the X, Y and Z position and orientation coordinates of the Storage
                linestream >> psx >> psy >> psz >> osx >> osy >> osz;
                storagePose = GetPoseFromString(std::stof(psx), std::stof(psy), std::stof(psz), std::stof(osx), std::stof(osy), std::stof(osz));

                // Get the X, Y and Z position and orientation coordinates where the produced Products will be spawned when requested
                linestream >> ppx >> ppy >> ppz >> opx >> opy >> opz;
                productOutputPose = GetPoseFromString(std::stof(ppx), std::stof(ppy), std::stof(ppz), std::stof(opx), std::stof(opy), std::stof(opz));

                // Add a new Storage to the Storages Vector
                storages.push_back(std::make_shared<Storage>(storageModel, productionModel, std::stoi(maxCapacity), storagePose, productOutputPose, modelController));
            }
        }
        storagesConfigFs.close();
    }
    // If any problem/exception happened, then return false to abort the simulation
    catch (const std::exception &e)
    {
        // Convert exception to a readable string and print a message
        std::ostringstream s;
        s << e.what();
        Print("Error while instantiating a Dispatch: " + s.str());
        return false;
    }

    // Add Dispatches from dispatches configuration file
    try
    {
        // Read Dispatches configuration file
        std::ifstream dispatchesConfigFs(projectDirectory + "/configs/dispatches");
        if (dispatchesConfigFs.is_open())
        {
            while (std::getline(dispatchesConfigFs, line))
            {
                // if line started with #, then discard this line
                // # is used as user configuration description
                if (line[0] == '#')
                {
                    continue;
                }
                std::istringstream linestream(line);
                std::string dispatchModel{};
                geometry_msgs::Pose dispatchPose{};
                geometry_msgs::Pose productPickPose{};
                std::string pdx{};
                std::string pdy{};
                std::string pdz{};
                std::string odx{};
                std::string ody{};
                std::string odz{};
                std::string ppx{};
                std::string ppy{};
                std::string ppz{};
                std::string opx{};
                std::string opy{};
                std::string opz{};

                // Get the Dispatch model name
                linestream >> dispatchModel;

                // Get the X, Y and Z position and orientation coordinates of the Dispatch
                linestream >> pdx >> pdy >> pdz >> odx >> ody >> odz;
                dispatchPose = GetPoseFromString(std::stof(pdx), std::stof(pdy), std::stof(pdz), std::stof(odx), std::stof(ody), std::stof(odz));

                // Get the X, Y and Z position and orientation coordinates where the produced Products will be spawned when requested
                linestream >> ppx >> ppy >> ppz >> opx >> opy >> opz;
                productPickPose = GetPoseFromString(std::stof(ppx), std::stof(ppy), std::stof(ppz), std::stof(opx), std::stof(opy), std::stof(opz));

                // Add a new Dispatch to the Dispatches Vector
                dispatches.push_back(std::make_shared<Dispatch>(dispatchModel, dispatchPose, productPickPose, modelController));
            }
        }
        dispatchesConfigFs.close();
    }
    // If any problem happened, then return false to abort the simulation
    catch (const std::exception &e)
    {
        // Convert exception to a readable string and print a message
        std::ostringstream s;
        s << e.what();
        Print("Error while instantiating a Dispatch: " + s.str());
        return false;
    }

    Print("Storages created: " + std::to_string(storages.size()));
    Print("Dispatches created: " + std::to_string(dispatches.size()));

    // If both Storages and Dispatches were instantiated properly, then return true
    // If storages and/or dispaches are empty, then return false
    return (storages.size() > 0 && dispatches.size() > 0) ? true : false;
}

int main(int argc, char **argv)
{
    // Initialize the WarehouseSimulation node and create a handle to it
    // NoSigintHandler option is need to the process of clear spawned objects in gazebo before this ros node completely shutdown
    ros::init(argc, argv, "WarehouseSimulation", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    // Set the project directory
    std::string projectDirectory{"../catkin_ws/src/warehouse_robot_simulation"};

    // Create an instance of the modelController
    std::shared_ptr<ModelController> modelController = std::make_shared<ModelController>("ModelController");

    // Create and instance of the orderController
    std::shared_ptr<OrderController> orderController = std::make_shared<OrderController>("OrderController");

    // Create Storage objects containers
    std::vector<std::shared_ptr<Storage>> storages;
    std::vector<std::shared_ptr<Dispatch>> dispatches;
    std::vector<std::shared_ptr<Robot>> robots;

    // Load models
    LoadModels(modelController, projectDirectory);

    // Instantiate Storage and Dispatch objects from config file
    // If any exception happened, end this simulation
    bool readConfigFiles = InstatiateWarehouseObjects(storages, dispatches, modelController, projectDirectory);
    if (!readConfigFiles)
    {
        Print("Please, check your configuration files and try again");
        // finish this ros node
        ros::shutdown();
        return 0;
    }

    // Create a Robot Object and configure it. Robot has to be already spawned in Gazebo and its AMCL node running properly
    robots.emplace_back(std::make_shared<Robot>("amr", "move_base", storages, dispatches, orderController));

    // Spawn each Storage in simulation and start it's operation
    std::for_each(storages.begin(), storages.end(), [modelController](std::shared_ptr<Storage> &s)
                  {
                      modelController->Spawn(s->GetName(), s->GetModelName(), s->GetPose());
                      s->StartOperation();
                  });

    // Spawn each Dispatch in simulation
    std::for_each(dispatches.begin(), dispatches.end(), [modelController](std::shared_ptr<Dispatch> &d)
                  { modelController->Spawn(d->GetName(), d->GetModelName(), d->GetPose()); });

    // Start each Robots operation
    std::for_each(robots.begin(), robots.end(), [modelController](std::shared_ptr<Robot> &r)
                  { r->StartOperation(); });

    ros::Subscriber ordersSubscriber = n.subscribe("warehouse/order/add", 10, &OrderController::AddOrder, &*orderController);

    // Set a sigHandler to handle CRTL+C
    signal(SIGINT, SigintHandler);

    // Keep running ros spinOnce at 10hz and until isShutdown is not set true by SIGINT handler
    ros::Rate loop_rate(10);
    while (ros::ok() && !isShutdown)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Start the shutting down process by deleting each spawned object from simulation
    std::for_each(storages.begin(), storages.end(), [modelController](std::shared_ptr<Storage> &s)
                  { modelController->Delete(s->GetName()); });

    std::for_each(dispatches.begin(), dispatches.end(), [modelController](std::shared_ptr<Dispatch> &d)
                  { modelController->Delete(d->GetName()); });

    // Check if there is any Product in the robots and delete it
    std::for_each(robots.begin(), robots.end(), [modelController](std::shared_ptr<Robot> &r)
                  {
                      std::vector<std::string> productsName = r->GetCargoBinProductsName();
                      if (productsName.size() > 0)
                      {
                          // Iterate over productsName and request modelController Delete
                          for (auto &p : productsName)
                          {
                              modelController->Delete(p);
                          }
                      }
                  });

    // Wait a bit more to guarantee the modelController delete requests be done
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // After Delete spawned models in Gazebo, then completely shutdown this ros node
    ros::shutdown();
    return 0;
}