#include "Storage.h"
#include "Product.h"
#include "WarehouseObject.h"
#include "Model.h"

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <std_msgs/String.h>

Storage::Storage(std::string modelName, std::string productModelName, geometry_msgs::Pose storagePose, geometry_msgs::Pose productOutputPose, std::shared_ptr<Model> modelController)
{
    _storageModelName = modelName;
    _storageName = modelName + "#" + std::to_string(_id);
    _objectName = modelName + "#" + std::to_string(_id);
    _productionModelName = productModelName;
    _storagePose = storagePose;
    _productOutputPose = productOutputPose;
    _modelController = modelController;
}

// Clear _productionModelName to safely finish production thread
Storage::~Storage()
{
    Print("Destructor");
    _productionModelName.clear();
}
std::string Storage::GetName()
{
    return _storageName;
}
std::string Storage::GetModelName()
{
    return _storageModelName;
}

std::string Storage::GetProductionModelName()
{
    return _productionModelName;
}

void Storage::SetProductionModel(std::string productModelName)
{
    _productionModelName = productModelName;
}

geometry_msgs::Pose Storage::GetPose()
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

std::unique_ptr<Product> Storage::RequestProduct()
{
    Print(GetProductionModelName() + " requested");

    std::lock_guard<std::mutex> lck(_storageMtx);
    if (_storedProducts.size() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_ptr<Product> product = std::move(_storedProducts.back());
        _storedProducts.pop_back();
        _modelController->Spawn(product->GetName(), product->GetModelName(), _productOutputPose);
        return std::move(product);
    }
    
    else
    {
        return nullptr;
    }
}

void Storage::StartOperation()
{
    // TODO: gazebo spawn here and delete on destructor
    threads.emplace_back(std::thread(&Storage::Production, this));
}

void Storage::Production()
{
    Print("Start " + _productionModelName + " production");
    while (!_productionModelName.empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // TODO define proper timing
        std::lock_guard<std::mutex> lck(_storageMtx);
        if (_storedProducts.size() < _maxCapacity)
        {
            std::unique_ptr<Product> product(new Product(_productionModelName));
            _storedProducts.push_back(std::move(product));
            Print(_storedProducts.back()->GetName() + " was produced - Total storage size is [" + std::to_string(_storedProducts.size()) + "]");
        }
    }
}