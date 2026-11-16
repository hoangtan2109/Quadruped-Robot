# 🐾 Autonomous Quadruped Robot (NotSpot-based Architecture)

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](http://gazebosim.org/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%20%2F%2F%20Ubuntu%2020.04-lightgrey.svg)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A complete hardware and software framework for a 12-DOF low-cost quadruped robot. This repository hosts the capstone engineering thesis project focusing on autonomous navigation, closed-loop posture stabilization via IMU feedback, and real-time kinematic gait generation using ROS (Robot Operating System) and Gazebo simulation.

---

## 🌟 Key Features

* **Kinematic Locomotion & Gait Generation:** Implements Trot and Stand gaits using analytical Inverse Kinematics (IK) and Bezier curve trajectory generators.
* **Real-time Posture Stabilization:** Integrated P-controller loop utilizing BNO055 IMU orientation feedback to dynamically compensate for Roll and Pitch errors during locomotion.
* **Autonomous Navigation Stack:** Full integration with Gmapping (SLAM) for map building, AMCL for adaptive Monte Carlo localization, and TEB Local Planner for optimal trajectory tracking and autonomous patrol.
* **Robust Odometry Estimation:** Utilizes Point-to-Line ICP (`laser_scan_matcher`) for accurate scan-matching and state estimation, eliminating wheel-slip drift common in standard LIDAR-inertial setups.
* **Embedded Hardware Interface:** Low-level control framework communicating via I2C to PCA9685 PWM drivers, managing 12 DS3235SG servos powered by a custom 20A Buck converter architecture.

---

## 📂 Workspace Structure (`Quadruped_ws`)

```tree
Quadruped_ws/
├── src/
│   ├── camera_capture_node/             # Camera video/image acquisition node
│   ├── my_tf_listener_pkg/              # C++ TF listener for trajectory tracking & odometry metrics
│   ├── Quadruped/                       # Core bringup package, drivers, and system launch files
│   ├── Quadruped_control/               # High/low-level controllers, gait schedulers, & IMU P-controller
│   ├── Quadruped_description/           # URDF/Xacro robot models, 3D meshes, and kinematic parameters
│   ├── Quadruped_gazebo/                # Gazebo simulation environments, worlds, & navigation configs
│   ├── Quadruped_hardware_interfacing/  # I2C PCA9685 servo interfacing & keyboard/joystick inputs
│   ├── Quadruped_simulation/            # Simulation utilities and testing scripts
│   └── Quadruped_utilities/             # Data logging, plotting tools, & patrol waypoint managers
├── Bezier_Trajectory.png                # Swing leg trajectory visualization
├── Homing_Response.png                  # System homing & stabilization response plot
└── TEB_Elastic_Band.png                 # TEB local planner trajectory optimization plot
```

---

## 🛠️ Installation & Build

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install ros-noetic-rosserial-python ros-noetic-rosserial-arduino ros-noetic-joy
```

### 2. Build & Source Workspace
```bash
cd ~/Quadruped_II/Quadruped_ws
catkin_make
source devel/setup.bash
```

---

## 💻 Simulation Guide (Gazebo)

### A. Manual Teleoperation
* **Terminal 1 (Launch Gazebo):**
  ```bash
  killall -9 gzserver gzclient roscore rosmaster 2>/dev/null
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  roslaunch quadruped quadruped_gazebo_sim.launch
  ```
* **Terminal 2 (Run Quadruped Driver):**
  ```bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  rosrun quadruped quadruped_driver.py 1 0 0
  ```
* **Terminal 3 (Keyboard Controller):**
  ```bash
  source ~/Quadruped_II/Quadruped_ws/devel/setup.bash
  python3 ~/Quadruped_II/Quadruped_ws/src/Quadruped_hardware_interfacing/quadruped_input_interfacing/scripts/Keyboard.py
  ```

### B. Autonomous Navigation & Patrol
* **Terminal 1 (Gazebo):**
  ```bash
  source /opt/ros/noetic/setup.bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  roslaunch quadruped quadruped_gazebo_sim.launch
  ```
* **Terminal 2 (Driver):**
  ```bash
  source /opt/ros/noetic/setup.bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  rosrun quadruped quadruped_driver.py 1 0 0
  ```
* **Terminal 3 (Load Map & Navigation Stack):**
  ```bash
  source /opt/ros/noetic/setup.bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  roslaunch quadruped_gazebo navigation.launch
  ```
* **Terminal 4 (RViz Visualization):**
  ```bash
  source /opt/ros/noetic/setup.bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  rviz -d ~/Quadruped_II/Quadruped_ws/src/Quadruped_gazebo/rviz/navigation.rviz
  ```
* **Terminal 5 (Auto Driver):**
  ```bash
  source /opt/ros/noetic/setup.bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  rosrun quadruped_gazebo auto_driver.py
  ```
* **Terminal 6 (Autonomous Waypoint Patrol):**
  ```bash
  source /opt/ros/noetic/setup.bash
  cd ~/Quadruped_II/Quadruped_ws && source devel/setup.bash
  rosrun quadruped_gazebo patrol.py
  ```

---

## 🤖 Real Hardware Experiments

### 1. BNO055 IMU Wake-up & Calibration (NDOF Mode)
Run this script on the Raspberry Pi 4B before launching the main controller to switch the BNO055 IMU to NDOF fusion mode (`0x0C` at register `0x3D`) and calibrate until `Sys=3`:
```bash
python3 - << 'EOF'
import smbus2, time

bus = smbus2.SMBus(1)
bus.write_byte_data(0x29, 0x3D, 0x0C)
time.sleep(0.1)

print("NDOF mode enabled. Starting Calibration...")
print("Rotate the robot in 6 different orientations, holding each for ~3s")
print("Wait until Sys=3 to complete\n")

while True:
    try:
        cal = bus.read_byte_data(0x29, 0x35)
        sys_cal   = (cal >> 6) & 0x03
        gyro_cal  = (cal >> 4) & 0x03
        accel_cal = (cal >> 2) & 0x03
        mag_cal   = cal & 0x03
        print(f"Sys:{sys_cal} Gyro:{gyro_cal} Accel:{accel_cal} Mag:{mag_cal}", end="\r")
        if sys_cal == 3:
            print("\nCalibration complete!")
            break
        time.sleep(0.1)
    except Exception:
        time.sleep(0.1)
EOF
```

### 2. Multi-Machine ROS Network & LIDAR Check
Configure network variables on the client/workstation to communicate with the Raspberry Pi ROS Master (`192.168.1.12`):
```bash
source ~/Quadruped_II/Quadruped_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.12:11311
export ROS_IP=192.168.1.10
rostopic hz /scan
```

### 3. Bringup Physical Robot
* **Terminal 1 (Hardware Bringup with IMU Stabilization):**
  ```bash
  roslaunch quadruped quadruped.launch use_joystick:=0 use_keyboard:=0 use_imu:=1 serial_port:=/dev/ttyUSB0
  ```
* **Terminal 2 (Remote Keyboard Control):**
  ```bash
  source ~/Quadruped_II/Quadruped_ws/devel/setup.bash
  python3 ~/Quadruped_II/Quadruped_ws/src/Quadruped_hardware_interfacing/quadruped_input_interfacing/scripts/Keyboard.py
  ```

---

## 📊 Experimental Results

| Bezier Foot Trajectory | Homing & Stabilization Response | TEB Local Planner Optimization |
| :---: | :---: | :---: |
| ![Bezier Trajectory](Bezier_Trajectory.png) | ![Homing Response](Homing_Response.png) | ![TEB Elastic Band](TEB_Elastic_Band.png) |

* **Posture Control:** Roll and pitch errors are strictly bounded below $\pm 0.01\text{ rad}$ during active Trot locomotion.
* **Signal Filtering:** Moving Average (MA) and Exponential Moving Average (EMA, $\alpha = 0.2$) successfully eliminate high-frequency IMU vibration noise while keeping phase delay minimal.

---

## 👤 Author

* **Nguyễn Hoàng Tân**
* **Major:** Control and Automation Engineering
* **Institution:** Ho Chi Minh City University of Technology (HCMUT / Đại học Bách Khoa TPHCM)
* **GitHub:** [github.com/hoangtan2109](https://github.com/hoangtan2109)
* **LinkedIn:** [linkedin.com/in/tannguyen21092004](https://www.linkedin.com/in/tannguyen21092004)

---

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
