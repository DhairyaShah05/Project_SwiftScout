
# Setup Instructions for SwiftScout Project

This document provides step-by-step instructions to set up the development environment for the SwiftScout project.

## Prerequisites
Ensure the following prerequisites are met:

- **Operating System**: Ubuntu 22.04 (Jammy Jellyfish)
- **Disk Space**: At least 20GB of free space
- **ROS2 Version**: Humble Hawksbill
- **Dependencies**: OpenCV, Gazebo 11, Navigation2 (Nav2)

## Step 1: Install ROS2 Humble
1. **Update and Upgrade System Packages**:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. **Add ROS2 Repository**:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update
   ```

3. **Install ROS2 Humble Desktop**:
   ```bash
   sudo apt install ros-humble-desktop
   ```

4. **Source ROS2 Environment**:
   Add the following line to your `.bashrc` file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
   Then reload the shell:
   ```bash
   source ~/.bashrc
   ```

5. **Verify Installation**:
   ```bash
   ros2 --version
   ```

## Step 2: Install Additional ROS2 Packages
1. **Navigation2 (Nav2)**:
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

2. **Rviz2**:
   ```bash
   sudo apt install ros-humble-rviz2
   ```

## Step 3: Install Gazebo 11
1. **Install Gazebo**:
   ```bash
   sudo apt update
   sudo apt install gazebo11 libgazebo11-dev
   ```

2. **Verify Gazebo Installation**:
   ```bash
   gazebo
   ```

## Step 4: Install OpenCV
1. **Install OpenCV Development Libraries**:
   ```bash
   sudo apt install libopencv-dev python3-opencv
   ```

2. **Verify OpenCV Installation**:
   Create a simple Python script to check OpenCV:
   ```python
   import cv2
   print(cv2.__version__)
   ```

3. **Run the Script**:
   ```bash
   python3 test_opencv.py
   ```

## Step 5: Clone and Set Up Project Repository
1. **Clone the Repository**:
   ```bash
   git clone <repository_url>
   cd SwiftScout
   ```

2. **Install Project Dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**:
   ```bash
   colcon build
   ```

4. **Source the Workspace**:
   ```bash
   source install/setup.bash
   ```

## Step 6: Run the Simulation
1. **Launch the Multi-Robot Simulation**:
   ```bash
   ros2 launch Swiftscout swift_scout_main.launch.py
   ```

2. **Monitor with Rviz2**:
   Open Rviz2 to visualize robot paths and goals:
   ```bash
   rviz2
   ```


## Notes
- Ensure all dependencies are properly installed and verified before running the simulation.
- Refer to the project README for additional configuration or troubleshooting steps.
