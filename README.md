# **Robot Following White Object (ROS2 Project)**

This project demonstrates the development of a simple robot simulation in **Gazebo** using **ROS2**. The robot follows a white object in its environment automatically using a camera sensor and a **Proportional-Integral-Derivative (PID)** controller. The project also includes placeholder implementations for **Pathfinding (A-Star)**, **Localization**, and other advanced control concepts for future extension.

---

## **Table of Contents**

1. [Overview](#overview)
2. [Features](#features)
3. [Installation and Setup](#installation-and-setup)
4. [How to Run](#how-to-run)
5. [Lampiran](#lampiran)

---

## **Overview**

This project involves creating a simple mobile robot in Gazebo that can perform an action autonomously using **ROS2**. Specifically, the robot is equipped with:
- A **URDF** model defining its physical structure (box + wheels).
- A **camera sensor** to detect objects in its environment.
- A control system using **OpenCV** for object detection and **PID Controller** for smooth movement.

The robot is tasked to:
1. Detect a white object in its camera field of view.
2. Move towards the object automatically, maintaining stability using PID control.

---

## **Features**

- **Object Detection**: Uses OpenCV to identify and track white objects.
- **PID Control**: Ensures smooth and stable robot motion while following the object.
- **Camera Calibration**: Simulated camera for accurate object localization.
- **URDF Model**: A simple robot model with a camera sensor and two wheels.
- **Pathfinding (Placeholder)**: Includes a framework for implementing the A* algorithm.
- **Localization (Placeholder)**: Provides a framework for future localization using odometry or sensors.

---

## **Installation and Setup**

### **1. Prerequisites**
- Install ROS2 (Foxy/Humble recommended).
- Install Gazebo for simulation.
- Install required dependencies:

```bash
sudo apt update
sudo apt install ros-<ros-distro>-gazebo-ros-pkgs ros-<ros-distro>-vision-msgs python3-colcon-common-extensions
```

### **2. Clone the Repository**
```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone <repository-link>
```

### **3. Build the Workspace**
```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

---

## **How to Run**

### **1. Launch the Robot Model**
Start the Gazebo simulation with the robot's URDF model:
```bash
ros2 launch robot_description display.launch.py
```

### **2. Start the Control System**
Run the control nodes for object detection and movement:
```bash
ros2 launch robot_control control.launch.py
```

### **3. Place a White Object in Gazebo**
Use Gazebo's model editor to add a white box in front of the robot.

### **4. Observe the Robot**
The robot should automatically detect and follow the white object in the simulation.

---

## **Lampiran**

1. **Video Simulasi**: https://drive.google.com/file/d/1DgXku_ZmVICsMR7rcJ6ecJ6KJvxl4efD/view?usp=drive_link
   - Video mendemonstrasikan robot pengikut objek putih.
2. **Dokumentasi**: https://drive.google.com/file/d/1aQ1xXnB4ctdu4qDjvUxBbhaCQG5bjRhq/view?usp=drive_link
   - Berisi penjelasan dan alur kerja dalam bentuk PDF.
3. **GitHub Repository**: https://github.com/Camn0/URO_ROS2_Gazebo/
   - Semua source code, fail launch, dan konfigurasi.
