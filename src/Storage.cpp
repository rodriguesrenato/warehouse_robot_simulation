#include "Storage.h"
#include "Product.h"
// #include "WarehouseObject.h"
#include "Model.h"

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <std_msgs/String.h>

Storage::Storage(std::string modelName, geometry_msgs::Pose storagePose, geometry_msgs::Pose productOutputPose, std::shared_ptr<Model> modelController)
{
    _storageModel = modelName;
    _storageName = modelName + "-" + std::to_string(_id);
    _storagePose = storagePose;
    _productOutputPose = productOutputPose;
    _modelController = modelController;
}

Storage::~Storage()
{
    _productionModel.clear();
}
std::string Storage::GetName()
{
    return _storageName;
}

std::string Storage::GetProductionModel()
{
    return _productionModel;
}

void Storage::SetProductionModel(std::string productModel)
{
    _productionModel = productModel;
}

geometry_msgs::Pose Storage::getPose()
{
    return _storagePose;
}

geometry_msgs::Pose Storage::GetProductOutputPose()
{
    return _productOutputPose;
}

bool Storage::GetGazeboSpawnStatus()
{
    return _isSpawned;
}
void Storage::SetGazeboSpawnStatus(bool status)
{
    _isSpawned = status;
}

bool Storage::RequestProduct(std::string productName, int quantity)
{
    std::lock_guard<std::mutex> lck(_storageMtx);
    if (_storedProducts.size() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::unique_ptr<Product> product = std::move(_storedProducts.back());
        _storedProducts.pop_back();
        return _modelController->Spawn(product->GetName(), product->GetModelName(), _productOutputPose);
    }
}

void Storage::RequestProduct1(const std_msgs::String &str)
{
    std::cout << "RequestProduct1\n";
    std::lock_guard<std::mutex> lck(_storageMtx);
    std::cout << "RequestProduct1>>lock_guard\n";
    if (_storedProducts.size() > 0)
    {
        ros::Duration(1.0).sleep();
        std::unique_ptr<Product> product = std::move(_storedProducts.back());
        _storedProducts.pop_back();
        _modelController->Spawn(product->GetName(), product->GetModelName(), _productOutputPose);
    }
}

void Storage::Simulate()
{
    threads.emplace_back(std::thread(&Storage::Production, this));
}

void Storage::Production()
{
    std::cout << "Production Started!\n";
    while (!_productionModel.empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::lock_guard<std::mutex> lck(_storageMtx);
        if (_storedProducts.size() < _maxCapacity)
        {
            std::unique_ptr<Product> product(new Product(_productionModel));
            _storedProducts.push_back(std::move(product));
            std::cout << _storedProducts.back()->GetName() << " added to " << this->GetName() << "[" << _storedProducts.size() << "]" << std::endl;
        }
    }
}