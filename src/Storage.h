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
    Storage(std::string modelName, std::string productModelName, geometry_msgs::Pose storagePose, geometry_msgs::Pose productOutputPose, std::shared_ptr<Model> modelController);
    ~Storage();
    std::string GetName();
    std::string GetModelName();
    std::string GetProductionModelName();
    void SetProductionModel(std::string productModel);
    geometry_msgs::Pose GetPose();
    geometry_msgs::Pose GetProductOutputPose();
    bool GetGazeboSpawnStatus();
    void SetGazeboSpawnStatus(bool status);
    void RequestProduct1(const std_msgs::String &str);

    std::unique_ptr<Product> RequestProduct();
    void StartOperation();
    void Production();

private:
    std::string _storageName{};
    std::string _storageModelName{};
    std::string _productionModelName{};
    geometry_msgs::Pose _storagePose;
    geometry_msgs::Pose _productOutputPose;
    bool _isSpawned{false};
    std::shared_ptr<Model> _modelController;

    int _maxCapacity{4};
    std::vector<std::unique_ptr<Product>> _storedProducts;
    std::vector<std::thread> _threads;
    std::mutex _storageMtx;
};

#endif