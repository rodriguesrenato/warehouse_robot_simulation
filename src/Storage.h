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

class Storage : public WarehouseObject
{
public:
    Storage(geometry_msgs::Pose pose);
    std::string GetName();
    bool RequestProduct(std::string productName, int quantity);
    void Simulate(ProductType _productType);
    void Production(ProductType _productType);

private:
    std::string _storageId;
    int _maxCapacity;  
    geometry_msgs::Pose _storagePosition;
    geometry_msgs::Pose _ProductOutputPosition;
    std::vector<std::unique_ptr<Product>> _storedProducts;
    std::vector<std::thread> _threads;
    std::mutex _storageMtx; 
};

#endif