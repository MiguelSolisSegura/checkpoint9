# Warehouse Robotics with ROS2: From Compiled Nodes to Components

This repository demonstrates the evolution of a robotics project using ROS2, focusing on an RB1 robot's journey to autonomously navigate, detect, and securely attach to a warehouse shelf within a simulated environment. The project transitions from a *Compiled Node Approach* to a more advanced *Component Approach*, showcasing a shift towards modularity and efficiency.

A more comprehensive explanation of this project can be found in:
- [https://miguelsolissegura.com/project/shelf-attachment](https://miguelsolissegura.com/project/shelf-attachment)
- [https://miguelsolissegura.com/project/ros2-components](https://miguelsolissegura.com/project/ros2-components)

## Prerequisites

Before you start, ensure the following prerequisites are met:

- ROS2 Galactic
- Gazebo simulation environment
- ROS1 (for RB1 robot compatibility)
- ROS1 Bridge for ROS2

## Installation

To get started, clone this repository into the `src` directory of your ROS2 workspace:

```
cd ~/ros2_ws/src
git clone https://github.com/MiguelSolisSegura/warehouse_robotics.git
```

## Setting Up the Environment

Prepare your environment by sourcing the required setup files and launching both the simulation and the ROS1 bridge. The setup differs slightly between the two approaches:

### Compiled Node Approach

1. **Launch the Warehouse Simulation** (ROS1 environment):
    ```
    source ~/simulation_ws/devel/setup.bash
    roslaunch rb1_base_gazebo warehouse_rb1.launch
    ```

2. **Launch the ROS1 Bridge**:
    ```
    source ~/catkin_ws/devel/setup.bash
    source /opt/ros/galactic/setup.bash
    ros2 run ros1_bridge parameter_bridge
    ```

### Component Approach

Before proceeding, ensure the Compiled Node Approach has been successfully implemented. The Component Approach builds upon the foundation laid by the previous phase.

## Running the Projects

### Compiled Node Approach

- **Pre-Approach Motion**:
    ```
    ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90
    ```
    This initiates the robot's movement towards the shelf.

- **Final Approach and Attachment**:
    ```
    ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.3 degrees:=-90 final_approach:=true
    ```

### Component Approach

- **Building and Sourcing**:
    ```
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

- **Running the Components**:
    First, start the component container:
    ```
    ros2 run rclcpp_components component_container
    ```
    Then, load the `PreApproach` component:
    ```
    ros2 component load /ComponentManager my_components my_components::PreApproach
    ```
    For the `AttachServer` and `AttachClient` components, use the following commands to load them within your container:
    ```
    ros2 component load /my_container my_components my_components::AttachServer
    ros2 component load /my_container my_components my_components::AttachClient
    ```
    Finally, launch the entire setup with:
    ```
    ros2 launch my_components attach_to_shelf.launch.py
    ```

## Additional Commands

For manual control of the RB1 robot or to perform specific task testing, utilize the ROS2 teleoperation package:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```

Focus the terminal window on the teleop command to navigate the robot using keyboard inputs.
