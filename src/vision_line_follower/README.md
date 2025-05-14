# Vision Line Follower - ROS 2 Humble Project

A ROS 2 project for a **vision-guided line follower robot** that can follow lines and scan QR codes to make decisions.

---

## 📦 Project Structure

- `src/vision_line_follower/`: ROS 2 package for the line follower
- `Line_follower.py`: Main logic for following lines and detecting QR codes
- `launch/`: Launch files for simulation in gazebo and RViz
- `urdf/`: Robot model
- `rviz/`: Predefined RViz configurations
---

## ⚙️ Setup & Launch Instructions

### ✅ Prerequisites

- Ubuntu 22.04
- ROS 2 Humble installed and sourced
- Python 3
- `colcon` build system

### 🛠️ Build Instructions

```bash
# Clone the repository
git clone https://github.com/Yousuf-Creater/vision_line_follower_ws.git
cd vision_line_follower_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source workspace
source install/setup.bash

# To spawn the robot and run simulation:
ros2 launch vision_line_follower spawn_launch.py

# To launch robot in RViz:
ros2 launch vision_line_follower rviz_launch.py
```

### ✅ Team Roles & Responsibilities
| Team Member      | Role                                    |
| ---------------- | --------------------------------------- |
| Muhammad Yousaf  | Developed the Line_follower node |
| Ahad Fayyaz      | Developed the robot URDF |
| Fraz Rehman      | Developed the Line_follower node |
| Ali Haider       | Developed the world file |



### 🛠️ ROS 2 Dependencies / Packages Used

Dependencies
Ensure the following dependencies are installed:
```
sudo apt update
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rclpy \
  python3-opencv
```

Additionally, install the pyzbar library (for QR code detection):
```
pip install pyzbar
```
Note: You may also need zbar backend:
```
sudo apt install libzbar0

```
Custom Packages
This project contains one main custom package:
vision_line_follower: Contains all logic for image capture, line detection, QR code scanning, and decision-making logic.

Working:

rclpy: ROS 2 Python client library

cv_bridge: OpenCV and ROS integration

pyzbar: for detection and decoding of qr codes 

sensor_msgs, geometry_msgs: Standard ROS message types

gazebo_ros: Gazebo simulation integration

urdf, xacro: For robot model

launch, launch_ros: For launching multiple nodes

vision_line_follower: Your custom package



