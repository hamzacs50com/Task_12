# Task 12 – Transformations and Omni Kinematics

This repository contains three parts:
1. **Individual Task** – Omni Wheels Kinematics  
2. **Task 12.1** – Transformations  
3. **Task 12.2** – Robot URDF and Visualization  

---

## 1. Individual Task – Omni Wheels Kinematics

This part implements the kinematics of a 4-wheel omni-directional robot base.  
The node subscribes to `/cmd_vel` and publishes the calculated wheel angular velocities on `/wheel_speeds`.

### Package Structure

omni_kinematics/
├── CMakeLists.txt
├── package.xml
├── launch/
│ └── omni_kinematics.launch
└── scripts/
└── omni_node.py


### Build
```bash
cd ~/catkin_ws/src
git clone <repo_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash

Run

Open 4 terminals:

    Terminal 1 – roscore

roscore

Terminal 2 – launch the omni kinematics node

roslaunch omni_kinematics omni_kinematics.launch

Terminal 3 – publish velocity commands

rostopic pub -r 5 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"

Terminal 4 – echo wheel speeds

    rostopic echo /wheel_speeds

Expected Output

The node converts /cmd_vel into four wheel angular velocities.
Example:

data: [8.0, 12.0, 8.0, 12.0]

Example Calculation

Parameters:

    L = 0.3 m

    W = 0.2 m

    R = 0.05 m

    Command: Vx = 0.5 m/s, Vy = 0.0 m/s, Wz = 0.2 rad/s

Steps:

    L + W = 0.5

    (L+W) * Wz = 0.1

    1/R = 20

Results:

    w1 = 20 * (0.5 - 0.0 - 0.1) = 8.0 rad/s

    w2 = 20 * (0.5 + 0.0 + 0.1) = 12.0 rad/s

    w3 = 20 * (0.5 + 0.0 - 0.1) = 8.0 rad/s

    w4 = 20 * (0.5 - 0.0 + 0.1) = 12.0 rad/s

Output matches:

[8.0, 12.0, 8.0, 12.0]

2. Task 12.1 – Transformations

This task demonstrates the use of transformations (TF) in ROS.
Build

cd ~/catkin_ws/src
git clone <repo_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash

Run

Open 3 terminals:

    Terminal 1 – roscore

roscore

Terminal 2 – launch the transformation setup

roslaunch transformations task12_1.launch

Terminal 3 – open RViz

    rviz

Notes

    The launch file loads the URDF and TFs.

    Check the TF tree with:

    rosrun rqt_tf_tree rqt_tf_tree

3. Task 12.2 – Robot URDF and Visualization

This task contains the robot description (URDF/Xacro) and a launch file for visualization in RViz.
Build

cd ~/catkin_ws/src
git clone <repo_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash

Run

Open 3 terminals:

    Terminal 1 – roscore

roscore

Terminal 2 – launch the robot model

roslaunch my_robot display.launch

Terminal 3 – open RViz if not already started

    rviz

Notes

    Modify the URDF/Xacro file to add sensors or links.

    To view the TF tree:

rosrun tf view_frames
evince frames.pdf
