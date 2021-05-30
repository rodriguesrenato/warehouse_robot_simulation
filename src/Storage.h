#ifndef STORAGE_H
#define STORAGE_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <geometry_msgs/Pose.h>

#include "Product.h"
#include "WarehouseObject.h"
#include "ModelController.h"

class Storage : public WarehouseObject
{
public:
    Storage(std::string modelName, std::string productModelName, int maxCapacity, geometry_msgs::Pose storagePose, geometry_msgs::Pose productOutputPose, std::shared_ptr<ModelController> modelController);
    ~Storage();
    std::string GetModelName();
    std::string GetProductionModelName();
    geometry_msgs::Pose GetPose();
    geometry_msgs::Pose GetProductOutputPose();
    std::unique_ptr<Product> RequestProduct();
    void StartOperation();

private:
    std::string _storageModelName{};                       // Storage model name
    std::string _productionModelName{};                    // Product model name that will be produced
    geometry_msgs::Pose _storagePose;                      // Storage Pose
    geometry_msgs::Pose _productOutputPose;                // Pose that Products will be delivered on RequestProduct call
    std::shared_ptr<ModelController> _modelController;     // Gazebo model controller
    int _maxCapacity;                                      // Maximum number of Products stored in this Storage
    std::mutex _storageMtx;                                // Mutex to access _storedProducts vector
    std::vector<std::unique_ptr<Product>> _storedProducts; // Store all produced Products

    void Production();
};

#endif