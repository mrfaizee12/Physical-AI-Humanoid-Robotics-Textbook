# Quickstart: Digital Twin (Gazebo & Unity)

**Feature**: 001-digital-twin-gazebo-unity
**Date**: 2025-12-09

## Overview

This quickstart guide will help you set up and run your first digital twin simulation using Gazebo and Unity for humanoid robot simulation. This guide covers the basic setup for physics-accurate simulation, visualization, and sensor data generation.

## Prerequisites

- ROS 2 (Humble Hawksbill or later)
- Gazebo Garden or Fortress
- Unity 2022.3 LTS or later
- Python 3.8+
- Node.js 18+ (for Docusaurus documentation)

## Installation

### 1. Clone the Repository

```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Install Dependencies

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs

# Install Python dependencies
pip install -r requirements.txt

# Install Node.js dependencies for documentation
cd book
npm install
```

### 3. Set Up Gazebo Environment

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select digital_twin_gazebo

# Source the workspace
source install/setup.bash
```

## Basic Digital Twin Setup

### 1. Launch Gazebo Physics Simulation

```bash
# Start the Gazebo simulation with humanoid robot
ros2 launch digital_twin_gazebo launch_simulation.launch.py
```

This will:
- Load the humanoid robot model into Gazebo
- Initialize physics with gravity and collision detection
- Set up the simulation environment

### 2. Connect Unity Visualization

1. Open Unity and load the Digital Twin project
2. Configure the Unity-ROS bridge connection
3. Run the Unity scene to visualize the robot

### 3. Configure Sensor Simulation

```bash
# Launch sensor simulation pipeline
ros2 launch digital_twin_gazebo launch_sensors.launch.py
```

This will:
- Enable LiDAR sensor simulation
- Configure depth camera simulation
- Activate IMU simulation
- Begin generating sensor data outputs

## Running Your First Simulation

### 1. Basic Physics Test

```bash
# Run a simple physics test
ros2 run digital_twin_gazebo physics_test.py
```

This test will:
- Place the humanoid robot in the Gazebo environment
- Enable gravity simulation
- Verify collision detection
- Output results to console

### 2. Visualization Test

1. In Unity, open the `DigitalTwinScene.unity`
2. Run the scene to visualize the robot
3. Verify that physics changes in Gazebo are reflected in Unity

### 3. Sensor Data Generation

```bash
# Generate sample sensor data
ros2 run digital_twin_gazebo generate_sensor_data.py
```

This will:
- Generate point cloud data from LiDAR simulation
- Create depth map data from camera simulation
- Output IMU readings
- Save data in standard formats

## Documentation Structure

The complete documentation is organized as follows:

1. **Gazebo Physics Simulation**: Gravity, collision, and environment setup
2. **Unity Interaction & Rendering**: Animation, lighting, and human-robot scenes
3. **Sensor Simulation Pipeline**: LiDAR, depth, and IMU data generation

## Reproducible Examples

All examples in this documentation are designed to be reproducible using Spec-Kit and Claude Code:

```bash
# Run any example with the reproducibility script
./scripts/reproduce-example.sh <example-name>
```

## Troubleshooting

### Common Issues

1. **Gazebo fails to start**
   - Check ROS 2 installation and environment sourcing
   - Verify GPU drivers and graphics support

2. **Unity-ROS connection fails**
   - Ensure both systems are on the same network
   - Check firewall settings for ROS communication

3. **Sensor data not generating**
   - Verify sensor plugins are loaded in Gazebo
   - Check that the sensor simulation node is running

## Next Steps

1. Complete the full Gazebo Physics Simulation chapter
2. Explore Unity Interaction & Rendering techniques
3. Learn about Sensor Simulation Pipeline implementation
4. Build your own humanoid robot simulation

## Support

For technical support, refer to the official documentation:
- Gazebo: http://gazebosim.org/
- Unity: https://docs.unity3d.com/
- ROS 2: https://docs.ros.org/