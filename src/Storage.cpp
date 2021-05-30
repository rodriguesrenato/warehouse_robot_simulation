#include "Storage.h"

// Define an unique Storage name and set class attributes
Storage::Storage(std::string modelName, std::string productModelName, int maxCapacity, geometry_msgs::Pose storagePose, geometry_msgs::Pose productOutputPose, std::shared_ptr<ModelController> modelController)
{
    _objectName = modelName + "#" + std::to_string(_id); // Create a unique storage name
    _type = ObjectType::objectStorage;                   // Set its object type
    _storageModelName = modelName;                       // Set its model Name
    _productionModelName = productModelName;             // Set the product model name to be in production
    _storagePose = storagePose;                          // Set the storage Pose
    _maxCapacity = maxCapacity;                          // Set the storage max capacity of stored Products
    _productOutputPose = productOutputPose;              // Set the product output Pose
    _modelController = modelController;                  // Set the modelController shared pointer
}

Storage::~Storage()
{
    Print("Shutting down");

    // Clear _productionModelName to safely finish production thread
    _productionModelName.clear();
}

// Return the Storage model name
std::string Storage::GetModelName()
{
    return _storageModelName;
}

// Return the Storage's Product model name that is set to be produced
std::string Storage::GetProductionModelName()
{
    return _productionModelName;
}

// Return Storage pose
geometry_msgs::Pose Storage::GetPose()
{
    return _storagePose;
}

// Return Storage's Product output pose
geometry_msgs::Pose Storage::GetProductOutputPose()
{
    return _productOutputPose;
}

// Spawn the Product model in Gazebo and return it's unique pointer
std::unique_ptr<Product> Storage::RequestProduct()
{
    std::lock_guard<std::mutex> lck(_storageMtx);

    // Check if a Product is available
    if (!_storedProducts.empty())
    {
        // Get a Product from _storedProducts, Spawn it in simulation and return it
        std::unique_ptr<Product> product = std::move(_storedProducts.back());
        _storedProducts.pop_back();
        _modelController->Spawn(product->GetName(), product->GetModelName(), _productOutputPose);
        return std::move(product);
    }
    else
    {
        // Otherwise, return a nullptr
        return nullptr;
    }
}

// Start Storage Production thread
void Storage::StartOperation()
{
    threads.emplace_back(std::thread(&Storage::Production, this));
}

// Adds a Product to _storedProducts on fixed time until Storage reach max capacity
void Storage::Production()
{
    Print("Starting " + _productionModelName + " production");

    // Keep producing Products until _productionModelName gets empty
    while (!_productionModelName.empty())
    {
        // Simulate a Product production time and avoid unnecessary processing
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        std::lock_guard<std::mutex> lck(_storageMtx);
        if (_storedProducts.size() < _maxCapacity)
        {
            // Add a new Product to _storedProducts
            std::unique_ptr<Product> product(new Product(_productionModelName));
            _storedProducts.push_back(std::move(product));
            // Print(_storedProducts.back()->GetName() + " was produced - Total storage size is [" + std::to_string(_storedProducts.size()) + "]");
        }
    }
    Print("Stopped production");
}