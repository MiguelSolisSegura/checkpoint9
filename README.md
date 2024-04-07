# Warehouse Robotics: Shelf Approach and Attachment Using ROS2

This project demonstrates the use of ROS2 in enabling an RB1 robot to autonomously navigate towards, detect, and securely attach to a warehouse shelf. The project is structured around ROS2 nodes and services, facilitating communication and control over the robot's actions within a simulated warehouse environment.

## Prerequisites

Before running the project, ensure you have the following installed:
- ROS2 Galactic
- Gazebo simulation environment
- ROS1 (for RB1 robot compatibility)
- ROS1 Bridge for ROS2

## Installation

Clone this repository into your ROS2 workspace's `src` directory. For example:

```
cd ~/ros2_ws/src
git clone https://github.com/MiguelSolisSegura/checkpoint9.git
```

## Setting Up the Environment

Before testing the project, you need to prepare your environment by sourcing the appropriate setup files and launching the simulation and ROS1 bridge. Here are the commands:

1. **Launch the Warehouse Simulation**:

For ROS1 environments:

```
source ~/simulation_ws/devel/setup.bash
roslaunch rb1_base_gazebo warehouse_rb1.launch
```

2. **Launch the ROS1 Bridge**:

To facilitate communication between ROS1 and ROS2 components:

```
source ~/catkin_ws/devel/setup.bash
source /opt/ros/galactic/setup.bash
ros2 run ros1_bridge parameter_bridge
```

## Running the Project

With the environment set up, you can proceed to test the project's functionality.

1. **Pre-Approach Motion**:

Navigate the robot to a position facing the shelf:

```
ros2 launch attach_shelf pre_approach.launch.xml obstacle:=0.3 degrees:=-90
```

This command initiates the robot's movement towards the shelf area, stopping and rotating as per the specified parameters.

2. **Final Approach and Attachment**:

To execute the final approach and shelf attachment:

```
ros2 launch attach_shelf attach_to_shelf.launch.py obstacle:=0.3 degrees:=-90 final_approach:=true
```

This command will run both the pre-approach motion and, upon successful completion, initiate the final approach to detect the shelf, position the robot underneath it, and secure the attachment.

## Additional Commands

For manual robot control or specific task testing, you can use ROS2's teleop package:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```

Ensure your terminal is focused on the teleop command window to control the robot using keyboard inputs.
