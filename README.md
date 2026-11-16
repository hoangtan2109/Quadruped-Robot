# 🐾 Autonomous Quadruped Robot (NotSpot-based Architecture)

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](http://gazebosim.org/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%20%2F%2F%20Ubuntu%2020.04-lightgrey.svg)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A complete hardware and software framework for a 12-DOF low-cost quadruped robot. This repository hosts the capstone engineering thesis project focusing on autonomous navigation, closed-loop posture stabilization via IMU feedback, and real-time kinematic gait generation using ROS (Robot Operating System) and Gazebo simulation.

---

## 🌟 Key Features

* **Kinematic Locomotion & Gait Generation:** Implements Trot and Stand gaits using analytical Inverse Kinematics (IK) and Bezier curve trajectory generators.
* **Real-time Posture Stabilization:** Integrated P-controller loop utilizing IMU orientation feedback to dynamically compensate for Roll and Pitch errors during locomotion.
* **Autonomous Navigation Stack:** Full integration with Gmapping (SLAM) for map building, AMCL for adaptive Monte Carlo localization, and TEB Local Planner for optimal trajectory tracking.
* **Robust Odometry Estimation:** Utilizes Point-to-Line ICP (`laser_scan_matcher`) for accurate scan-matching and state estimation, eliminating wheel-slip drift common in standard LIDAR-inertial setups.
* **Embedded Hardware Interface:** Low-level control framework communicating via I2C to PCA9685 PWM drivers, managing 12 DS3235SG servos powered by a custom 20A Buck converter architecture.

---

## 📂 Repository Structure

```tree
├── quadruped_gazebo/         # Gazebo simulation models, worlds, and URDF/Xacro files
├── quadruped_control/        # High-level and low-level control nodes (Python/C++)
│   ├── scripts/              # Controller logic, IK solver, and EMA filtering
│   └── src/                  # C++ hardware interface and TF tracking nodes
├── navigation_stack/         # Configuration files for Gmapping, AMCL, and TEB Planner
└── README.md
