Here is the README.md file for my project:  


TurtleBot Mapping in Gazebo

This project enables autonomous mapping using a TurtleBot equipped with LiDAR sensors in a Gazebo simulation environment. The robot navigates the world, avoiding obstacles while collecting mapping data.  

Prerequisites 

Before running the mapping process, ensure you have the following installed:  
- **Ubuntu 20.04** (or compatible version)  
- **ROS 2 (Foxy, Galactic, or Humble recommended)**  
- **Gazebo simulator**  
- **TurtleBot3 packages**  
- **Navigation and SLAM tools (if needed for mapping visualization)**  

 **Installing Required Dependencies**  

1. Install ROS 2:  
   
   sudo apt update && sudo apt install ros-foxy-desktop
   
2. Install Gazebo:  
   
   sudo apt install gazebo11
   
3. Install TurtleBot3 packages:  
   
   sudo apt install ros-foxy-turtlebot3*
   
4. Source ROS 2 in every new terminal:  
   
   source /opt/ros/foxy/setup.bash
   

 **How to Run the Mapping Process**  
  **1. Run the Mapping Node**  
Open a new terminal and docker and run the mapping script:  
cd ~/ros2_ws
ros2 launch my_robot_controller mapping.launch.py

 **2. Observe the Mapping Process**  
The TurtleBot will start moving and scanning its surroundings. It will avoid obstacles while exploring the environment.  

 **3. Save the Map (if using SLAM tools)**  
If you are using SLAM for mapping, save the generated map using:  
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/my_robot_controller/map/new_world

## **Troubleshooting**  

- **Gazebo does not start:** Ensure Gazebo is installed correctly and run `gazebo` separately to check for errors.  
- **Robot is not moving:** Make sure the mapping script is running and the `/cmd_vel` topic is publishing commands.  
- **Mapping is incomplete:** Check LiDAR sensor data and adjust movement logic if needed.

  I am sorry if this file will not help you a lot open this file. I never did it before so a tried to do it as logic as possible. If you have any problems lets find out the solution duriong our next session.



HERE IS README.file FOR TASK 3

# Task 3: Autonomous Navigation of Ego Vehicle Using Autoware

## Overview

This project demonstrates autonomous navigation of an ego vehicle using the **Autoware stack** in a simulated environment. A custom ROS 2 Python node sets the initial pose and sequentially publishes navigation goals, while also monitoring the vehicle’s position to determine when each goal is reached.

---


## Prerequisites

- ROS 2 (e.g., Foxy, Galactic, Humble)
- Autoware installed and sourced
- Functional planning simulator with sample map
- Docker (optional but recommended for Autoware compatibility)
- A built and sourced workspace

---

## Setup Instructions

1. **Clone the repository and build the workspace**:
   ```bash
   mkdir -p ~/autoware_ws/src
   cd ~/autoware_ws/src
   git clone <your-repository-link>
   cd ..
   colcon build --symlink-install
   source install/setup.bash
````

2. **Make the node executable**:

   ```bash
   chmod +x src/my_robot_controller/my_robot_controller/av_nav.py
   ```


## How to Launch

Run the following command to start the full navigation system:

```bash
ros2 launch my_robot_controller cav_nav.launch.py
```

This will:

* Launch the Autoware planning simulator with your custom map
* Launch your custom navigation node that:

  * Sets the vehicle's initial pose
  * Publishes a sequence of goals
  * Switches Autoware to autonomous mode

---

## Custom Navigation Node (`av_nav.py`)

* Publishes to `/initialpose` and `/planning/mission_planning/goal`
* Subscribes to `/localization/kinematic_state` to monitor progress
* Sends a service request to `/system/operation_mode/change_operation_mode` to enable autonomous driving
* Automatically proceeds to the next goal when the current one is reached

### Example Goal Coordinates Used

```python
self.goal_poses = [ 
  {'x': 3819.13, 'y': 73797.85, 'zz': -0.52, 'w': 0.85}, 
  {'x': 3701.32, 'y': 73762.91, 'zz': 0.229, 'w': 0.973},
  {'x': 3882.115, 'y': 73825.328, 'zz': 0.845, 'w': 0.5335}
]
```

> These coordinates were carefully selected and validated from the simulator. Incorrect initial values led to navigation issues.

---

## Challenges & Solutions

* **Incorrect goal coordinates**: Initially, some poses were logged inaccurately from the simulator. This caused the vehicle to fail navigation or stop unexpectedly. After carefully re-logging correct coordinates, the issue was resolved.
* **Docker version mismatch**: An older Docker image caused compatibility problems. After switching to the latest supported Autoware image, both Gazebo and RViz2 launched properly.

---

## Validation

✔ Vehicle starts at the specified initial pose
✔ Navigates autonomously through all waypoints
✔ Stops safely after completing the mission

---

