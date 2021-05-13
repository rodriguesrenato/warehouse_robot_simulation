#include "Storage.h"
#include "Product.h"

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

Storage::Storage(geometry_msgs::Pose pose)
{
    _storageId = "Storage-" + std::to_string(_id);
    _storagePosition = pose;
}

std::string Storage::GetName()
{
    return Storage::_storageId;
}

bool Storage::RequestProduct(std::string productName, int quantity)
{
    std::unique_ptr<Product> product = std::move(_storedProducts.back());
    _storedProducts.pop_back();
     
    product->Spawn(_storagePosition);
}

void Storage::Simulate(ProductType productType)
{
    threads.emplace_back(std::thread(&Storage::Production, this, productType));
}

void Storage::Production(ProductType productType)
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::lock_guard<std::mutex> lck(_storageMtx);
        if (_storedProducts.size() < _maxCapacity)
        {
            std::unique_ptr<Product> product = std::make_unique<Product>(productType);
            _storedProducts.emplace_back(product);
            std::cout << product->GetName() << "added to " << this->GetName() << std::endl;
        }
    }
}