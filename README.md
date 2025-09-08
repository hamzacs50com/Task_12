# Task 12 – Individual Submission  
Electrical Team Training 2025  

This document explains how to complete **Task 12.1 (Build Your Own Robot)** and **Task 12.2 (Transform Your Troops)**.  
It includes setup instructions, execution steps, expected outputs, and deliverables.  

---

## 📌 Task 12.1 – Build Your Own Robot  

### Objective
- Create a **4-wheel robot** in ROS using **URDF/XACRO**.  
- The robot should include:
  - Base link (main body)  
  - Four wheels (two left, two right)  
  - A LiDAR sensor mounted on top of the base link  
- Visualize the robot in **RViz** and spawn it in **Gazebo**.  

---

### Files Required
1. `ae86_bot.xacro` → robot description (URDF using XACRO macros)  
2. `spawn_robot.launch` → launch file to load the robot in Gazebo and RViz  
3. `my_robot.rviz` → pre-configured RViz settings (grid, TF, RobotModel)  

---

### Setup Instructions
1. **Create a ROS package**
   ```bash
   cd ~/catkin_ws/src
   catkin_create_pkg my_robot rospy roscpp std_msgs sensor_msgs urdf xacro robot_state_publisher gazebo_ros rviz
