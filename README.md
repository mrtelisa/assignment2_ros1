# Assignment 2 ROS1

This repository contains the code for the ROS1-based part of the 2nd assignment of the Research Track 1 course.

## Description

The project implements a control system for a mobile robot in a simulated environment (Gazebo). It includes two main Python nodes:

- **Robot Control Node**: This node manages the robot's movement by receiving velocity commands and sending corresponding commands to the motors.

- **Monitoring Node**: This node monitors the robot's state by collecting information from sensors and publishing data about the robot's current status. Those informations are written in a service, too.


## Prerequisites

- ROS Noetic

- Catkin workspace

- Clone this repository in the src folder of the workspace created

- Clone another repository, always in the src folder:
    ```bash
    git clone https://github.com/CarmineD8/assignment_2_2024
    ```

## Usage

1. **Launching the ROS Nodes**
    
    Launch the robot control and simulation:
    ```bash
    roslaunch ass2_ros1 ass2.launch
    ```
    
2. **Interacting in the shell**

    - The user can chose the target point where the robot should be headed to.

    - Also, while the robot is moving, the user can ask for a feedback (about position and velocity), or can quit or even exit the simulation.

