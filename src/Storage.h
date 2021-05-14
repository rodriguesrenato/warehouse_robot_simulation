#ifndef STORAGE_H
#define STORAGE_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

// #include "Product.h"
#include "WarehouseObject.h"
class Product;
class Model;

class Storage : public WarehouseObject
{
public:
    Storage(std::string modelName, geometry_msgs::Pose storagePose, geometry_msgs::Pose productOutputPose, std::shared_ptr<Model> modelController);
    ~Storage();
    std::string GetName();
    std::string GetProductionModel();
    void SetProductionModel(std::string productModel);
    geometry_msgs::Pose getPose();
    geometry_msgs::Pose GetProductOutputPose();
    bool GetGazeboSpawnStatus();
    void SetGazeboSpawnStatus(bool status);
    void RequestProduct1(const std_msgs::String &str);

    bool RequestProduct(std::string productName, int quantity);
    void Simulate();
    void Production();

private:
    std::string _storageName{};
    std::string _storageModel{};
    std::string _productionModel{};
    geometry_msgs::Pose _storagePose;
    geometry_msgs::Pose _productOutputPose;
    bool _isSpawned{false};
    std::shared_ptr<Model> _modelController;

    int _maxCapacity{10};
    std::vector<std::unique_ptr<Product>> _storedProducts;
    std::vector<std::thread> _threads;
    std::mutex _storageMtx;
};

#endif