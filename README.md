# **Robot Following White Object (ROS2 Project)**

This project demonstrates the development of a simple robot simulation in **Gazebo** using **ROS2**. The robot follows a white object in its environment automatically using a camera sensor and a **Proportional-Integral-Derivative (PID)** controller. The project also includes placeholder implementations for **Pathfinding (A*)**, **Localization**, and other advanced control concepts for future extension.

---

## **Table of Contents**

1. [Overview](#overview)
2. [Features](#features)
3. [Project Components](#project-components)
   - [Movement Parameters](#movement-parameters)
   - [Object Detection](#object-detection)
   - [Camera Calibration](#camera-calibration)
   - [Localization](#localization)
   - [Pathfinding Algorithm (A*)](#pathfinding-algorithm-a)
   - [Movement Signal Control](#movement-signal-control)
   - [Pose Estimation](#pose-estimation)
   - [Proportional Integral Derivative (PID)](#proportional-integral-derivative-pid)
   - [Active Disturbance Rejection Control (ADRC)](#active-disturbance-rejection-control-adrc)
4. [System Workflow](#system-workflow)
5. [Installation and Setup](#installation-and-setup)
6. [How to Run](#how-to-run)
7. [Deliverables](#deliverables)
8. [Future Work](#future-work)
9. [Acknowledgments](#acknowledgments)

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

## **Project Components**

### **Movement Parameters**
- **Linear Velocity (`cmd_vel.linear.x`)**: Controls forward/backward movement.
- **Angular Velocity (`cmd_vel.angular.z`)**: Controls rotation for aligning the robot with the object.
- These parameters are dynamically adjusted using the PID Controller.

---

### **Object Detection**
- **Approach**:
  - The camera captures images from the environment.
  - OpenCV processes the images to filter white-colored objects.
  - The object's position (center of the bounding box) is extracted for movement control.
- **Key Functions**:
  - `cv2.inRange()`: Filters white pixels based on HSV color space.
  - `cv2.findContours()`: Locates object boundaries in the filtered image.

---

### **Camera Calibration**
- **Purpose**:
  - Simulated calibration ensures the robot's camera provides accurate positional data.
- **Configuration**:
  - Horizontal Field of View (FOV): 90 degrees.
  - Resolution: 640x480 pixels.
  - Camera placement: Mounted on top of the robot, facing forward.

---

### **Localization**
- **Future Scope**:
  - Implement localization techniques using odometry or SLAM packages.
  - Use sensor fusion (IMU + Camera) for precise pose estimation.

---

### **Pathfinding Algorithm (A*)**
- **Placeholder**:
  - A* algorithm will be implemented for autonomous navigation in a structured environment.
  - It calculates the shortest path between the robot and a target goal using a grid-based map.

---

### **Movement Signal Control**
- Publishes velocity commands (`geometry_msgs/Twist`) to the `/cmd_vel` topic.
- Commands are computed dynamically based on object position and PID output.

---

### **Pose Estimation**
- **Placeholder**:
  - Future integration of techniques like Kalman Filter or Particle Filter for precise localization.

---

### **Proportional Integral Derivative (PID)**
- **Purpose**:
  - Smoothly controls the robot's motion towards the detected object.
- **Implementation**:
  - `Proportional (P)`: Reacts to the current error.
  - `Integral (I)`: Compensates for accumulated error over time.
  - `Derivative (D)`: Reduces overshooting by predicting future errors.

---

### **Active Disturbance Rejection Control (ADRC)**
- **Placeholder**:
  - A more advanced control technique for rejecting disturbances will be explored in future iterations.

---

## **System Workflow**

1. **Object Detection**:
   - The robot's camera captures images.
   - OpenCV detects white-colored objects and extracts their positions.
2. **PID Control**:
   - The PID controller calculates the error between the object position and the robot's current position.
   - Adjusts linear and angular velocities accordingly.
3. **Movement Execution**:
   - Velocity commands (`cmd_vel`) are sent to the robot's wheels, making it move towards the object.

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

## **Deliverables**

1. **Simulation Video**:
   - A video demonstrating the robot following a white object in Gazebo.
2. **Documentation**:
   - This README file explaining the system's components and workflow.
3. **GitHub Repository**:
   - Include all source code, launch files, and configurations.
