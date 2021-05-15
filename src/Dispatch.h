#ifndef DISPATCH_H
#define DISPATCH_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>


#include "WarehouseObject.h"
class Product;
class Model;

class Dispatch : public WarehouseObject
{
public:
    Dispatch(std::string modelName, geometry_msgs::Pose dispatchPose, std::shared_ptr<Model> modelController);
    ~Dispatch();

    std::string GetName();
    std::string GetModelName();
    geometry_msgs::Pose getPose();

    bool GetGazeboSpawnStatus();
    void SetGazeboSpawnStatus(bool status);

    bool PickProduct(std::unique_ptr<Product> product);

private:
    std::string _dispatchName{};
    std::string _dispatchModelName{};
    geometry_msgs::Pose _dispatchPose;

    bool _isSpawned{false};
    std::shared_ptr<Model> _modelController;

    std::vector<std::unique_ptr<Product>> _storedProducts; // need?
    std::vector<std::thread> _threads;
    std::mutex _dispatchMtx;
};

#endif