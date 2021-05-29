# warehouse_robot_simulation

This project consists of a automated warehouse simulation with an autonomous mobile robot that handles `Orders` requests to get the required `Products` at the `Storages` available and deliver them in the right `Dispatch` area.

> This project were developed to be presented as the "Capstone Project" of Udacity C++ Engineer Nanodegree program. I chose to develop this project in the mobile robotics context with everything I've learned from the Udacity Robotic Software Engineer Nanodegree Program, which I had the chance to develop a home service robot simulation (repo [here](https://github.com/rodriguesrenato/rse-nd-home-service-robot)) that navigates autonomously between two goals pose.

# Installation

# Usage

# Simulation

The simulation has the following components:
- Warehouse world in Gazebo
- URDF Robot
- AMCL and Navigation nodes
- Program C++ Classes
    - Robot
    - Storage
    - Dispatch
    - Product
    - Order
    - OrderController
    - ModelController

## Warehouse World

![Warehouse world](docs/images/warehouse_world.png)

The world was built in a simplified version of a warehouse and was designed in a way that Storages area will be at the right side of the image above and Dispatch areas on the left, so robot will have to navigate through aisles to get the products and deviler them on the left side of the open area, similar to some logistics warehouse configurations.

## URDF Robot

![URDF Robot](docs/images/urdf_robot.png)

The robot was designed in a two wheeled configuration with ball caster wheels at edges, a cargo bed at the back and two sensors for mapping and localization (camera and Lidar). 

It was built using URDF format and .xacro files, based and improved from my previous project [home_service_robot](https://github.com/rodriguesrenato/rse-nd-home-service-robot) by adding some macro functions to help adusting parameters and calculating inertial values.

## Mapping, Localization and Navigation nodes

These nodes were based on [home_service_robot](https://github.com/rodriguesrenato/rse-nd-home-service-robot) project.

SLAM `gmapping` package were used to build the warehouse map. The map is loaded with `map_server` package, the localization is done with `AMCL` package and navigation with `move_base` package.
Navigation parameters were based on the koburi/turtlebot navigation parameters as a starting point and they were tuned for this application. The `inflation_radius` param was increasead to avoid naviagation too close to the walls and ground objects


## WarehouseSimulation ROS node

This simulation was developed using ROS with C++ and Gazebo. All objects in the simulation are instantiated/handled in the warehouseSimulation node, except the robot and localization/navigation nodes that has to be launched previously. The diagram bellow shows the node main operation flow.

!(warehouse simulation diagram)[]

1. Initialize the WarehouseSimulation node and create the warehouse controllers and objects. 

2. All SDF models that will be used in the simulation are loaded and its file content stored in a dictionary of `modelname:fileContent` by the modelController class. The model files can be found at `models/warehouseObjects` folder.

3. Read the storages and dispatches configuration files to add Storage and Dispatch objects to the simulation. These objects are defined on each line in the configuration files. InstatiateWarehouseObjects function process these configuration files line by line, construct the correspondent object with the values read and push it back to the correspondent object's vector. If any exception/problem occurs during this process, the simulation is aborted.

4. Create a `Robot` object and configure it. The simulation was made to handler multiple robots, but in this current version just one robot is used. Check `Robot` constructor for more information about the parameter needed to set multiple `Robots`.

5. Iterate over each `Storage`, `Dispatch` and `Robot` created in order to Start Operation threads and/or spawn it's object model in Gazebo simulation. `Storage` Operation thread is responsible for keeping `Storage` producing `Products` to it's max capacity. `Robot` Operation thread is responsible for cycle through tasks to execute `Orders` and interact with all other warehouse objects.

6. Set a ROS subscriber at topic `warehouse/order/add` to receive `Order` requests and send it to `OrderController::AddOrder`. This subscribe function opens a new thread internally to call the AddOrder member function of `OrderController`, which is passed as reference.

7. Set a new `SIGINT` handler to let the simulation call `ModelController::Delete` of all models that were previously spawned before a complete ros shutdown. `ros::init` at step 1. was set with the flag `ros::init_options::NoSigintHandler` to let a custom handler be set.

8. A while loop keeps calling `ros::spinOnce()` to process a single round of ROS callbacks until global variable `isShutdown` is set or ROS shutdown.

9. When `isShutdown` is set, iterates over all Storage, Dispatch and Robot to call delete models from Gazebo simulation

10. After Delete spawned models in Gazebo, then completely shutdown this ros node by calling `ros::shutdown()`


## C++ Classes Structue

A brief explanation of each implemented class

### WarehouseObject

This is the base class for all objects and controller of this simulation. It is responsible for generating a unique id, printing messages in the terminal protected by a mutex through `Print()` member function, retrive it's unique name through `GetName()` and storing all started threads in a vector to build a thread barrier on it's Destructor. It was implemented a way of programatically end objects (`Storage` and `Robot`) member functions that were started in threads.

### Robot

In this simulation, a two wheeled mobile Robot is used. It has a cargo bed at the back to carry `Products`, a Lidar and camera sensors to localize itself in the enviroment and navigate, and it can interact with other `Warehouse Objects`.

The Robot class has a member functions to set/return it's status, return a list of Product names that is in the cargo bed and the StartOperation member function responsible for start a thread running `Operate()`. It also has private member functions that builds a vector of Storages that have the Products in the current order; interacts with SimpleActionClient to move the robot; and operate robot throught RobotStatus.

In the `Operate()` private member function, a state machine of `RobotStatus` was implemented, which runs continuously until _status be set as `offline`. Each `RobotStatus` is responsible for a task listed bellow. Some operation variables are created before the state machine scope to be persistent between states looping. It was made with this strategy to continuously check if the robot _status was set to `offline` and then terminate this thread. When `Robot` Destructor is called, it set _status to `offline`.

- `offline`: Robot is not operable and shutdown.

- `startup`: Initialize the SimpleActionClient and wait until it gets ready to change the _status to `requestOrder`.

- `standby`: Just wait until _status change.

- `requestOrder`: Request OrderController an Order and change _status to `processOrder` if got one.

- `processOrder`: Clear operation variables, get the list of Storages to get Order Products and get the target Dispatch shared pointer. If it couldn't find any Storages to go or a valid Dispatch, then set _state to `closeOrder`.

- `plan`: at this state, set the next target Storage, remove this Storage from storagesToGo vector and set _state to `moveToStorage`. If storagesToGo gets empty, then set _state to `moveToDispatch`.

- `moveToStorage`: Move robot to the Storage product output pose. If it reaches the goal, then set _status to `requestProduct`, otherwise try again in next state machine interation.

- `requestProduct`: Call `RequestProduct()` member function of the target Storage until the gets the number of valid Products specified in the Order. `RequestProduct()` return a **unique_ptr<Product>** that will be moved to Robot `_cargoBinProducts` attribute, passing Product's ownership from Storage to Robot. When the storage don't have a Product available, it will return a nullptr, that will not be counted and added to `_cargoBinProducts`.

- `moveToDispatch`: Move robot to the Dispatch product picking pose. If it reaches the goal, then set _status to `dispatchOrder`, otherwise try again in next state machine interation.

- `dispatchOrder`: Call target Dispatch `PickProduct()` member function moving Products in `_cargoBinProducts` one by one, passing Product's ownership from Robot to Dispatch, until `_cargoBinProducts` gets empty. After that, set _status to `closeOrder`.

- `closeOrder`: Close this order and set _state to `requestOrder` to request a new one.


### Storage

A unit that continuously produces a specified `Product` until gets max capacity. It handles `Products` requests to spawn a `Product` at a specified `Product Output Pose`


### Dispatch


This class runs one thread to keep `Product` production.

- Next Step: Documentation

### Product


### Order

### OrderController

### Model Controller

# Development Process

Plan the solution, design a first draft of the components, classes

Design the warehouse simplified map

Design the delivery robot with a container for the packages on the rear side

Develop a ROS node that spawn/remove gazebo objects in the world -> Test the package beeing picking and drop-off in the robot's container

Map the warehouse with the delivery robot

Test AMCL and Navigation stack, based on the final project of the Udacity Robotic Software Engineer Nanodegree

Define the picking points and model representation in the world

Define Drop-off points and model representation in the world

Build an UI screen in opencv to accept mouse clicks and show operation numbers

Build the warehouse controller node

# Next features

# License
The contents of this repository are covered under the MIT License.

# References

- how to add threads to makefile https://stackoverflow.com/questions/67300703/how-do-i-use-the-pthreads-in-a-ros-c-node
- Ros wiki (sigint)