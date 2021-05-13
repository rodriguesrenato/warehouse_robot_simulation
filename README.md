# delivery_robot_simulation

This project is under development.


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

# Class structures

 - Storage Controller
    - Output Package
    - Input Package
    - Status
        - Capacity

 - Delivery Robot Controller
    - Status
    - Container Cap
    - Move to Position
    - Request to Pick Package
    - Request to Deliver Package
    - Info
        - Max_cap
        - Get Container Position

- Operation Controller
    - Initialize Simualtion -> Button Click?
        - Load Warehouse Config File
        - Place Robot
        - Place the Storages Zones
        - Place Drop-off Zones
    - Storages Queue
    - Robot Queue
    - Order Queue
    - Planner -> take most recent order and send to robot

- Package controller
    - Spawn
        - Id, Pos xyz, color
    - Remove
        - Id

- Robot Controller
    - Tasks Queue
    - ProcessTasks
    - GetStatus
    - ClearTasks (remove objects)
    private:
    - ContainerObjects
    - Status
    - NextTask

# References

- https://roboticsbackend.com/ros-include-cpp-header-from-another-package/
- https://www.youtube.com/watch?v=1bnEdQzf8Yw