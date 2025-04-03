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

