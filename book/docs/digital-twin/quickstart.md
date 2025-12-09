---
sidebar_position: 5
title: "Quickstart Guide"
---

# Quickstart Guide: Digital Twin (Gazebo & Unity)

This quickstart guide will help you get up and running with the Digital Twin module focusing on physics-accurate digital twins using Gazebo and Unity for humanoid robot simulation.

## Prerequisites

Before starting with the Digital Twin module, ensure you have:

- **ROS 2 (Humble Hawksbill or later)** installed and configured
- **Gazebo Garden or Fortress** for physics simulation
- **Unity 2022.3 LTS or later** for visualization
- **Python 3.8+** for scripting and automation
- **Node.js 18+** for Docusaurus documentation site

## Getting Started

### 1. Repository Setup

First, clone the repository and set up the environment:

```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Install Dependencies

Install the required dependencies for the digital twin simulation:

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs

# Navigate to the book directory and install Node.js dependencies
cd book
npm install
```

### 3. Build the Workspace

Build the ROS 2 workspace with the digital twin packages:

```bash
cd ~/ros2_ws
colcon build --packages-select digital_twin_gazebo digital_twin_unity
source install/setup.bash
```

## Running Your First Simulation

### 1. Launch Gazebo Physics Simulation

Start with the physics simulation to test the foundational layer:

```bash
# Launch the basic physics simulation
ros2 launch digital_twin_gazebo launch_physics_demo.launch.py
```

This will:
- Load the humanoid robot model into Gazebo
- Initialize physics with gravity and collision detection
- Set up the simulation environment

### 2. Launch Unity Visualization

In a separate terminal, start the Unity visualization:

```bash
# Navigate to Unity project directory and launch
cd unity_projects/digital_twin_visualization
# Run Unity Editor or build executable
```

### 3. Run Sensor Simulation

Connect the sensor simulation pipeline:

```bash
# Launch sensor simulation
ros2 launch digital_twin_gazebo launch_sensors.launch.py
```

## Exploring the Documentation

The Digital Twin module is organized into three main chapters:

### 1. Gazebo Physics Simulation
Start with the physics foundation:
- Navigate to "Digital Twin (Gazebo & Unity) → Gazebo Physics Simulation"
- Learn about gravity configuration, collision detection, and environmental interactions
- Follow the reproducible examples to create your first physics simulation

### 2. Unity Interaction & Rendering
Explore visualization capabilities:
- Visit "Digital Twin (Gazebo & Unity) → Unity Interaction & Rendering"
- Discover how to create interactive scenes with high-fidelity rendering
- Practice with the human-robot interaction scenarios

### 3. Sensor Simulation Pipeline
Implement perception capabilities:
- Access "Digital Twin (Gazebo & Unity) → Sensor Simulation Pipeline"
- Understand how to generate realistic sensor data (LiDAR, depth, IMU)
- Apply sensor data to perception tasks

## Reproducible Examples

Each chapter includes reproducible examples that you can run to validate your setup:

1. **Physics Demo**: `ros2 launch digital_twin_gazebo launch_physics_demo.launch.py`
2. **Unity Visualization**: `ros2 launch digital_twin_unity launch_visualization_demo.launch.py`
3. **Sensor Pipeline**: `ros2 launch digital_twin_gazebo launch_sensor_demo.launch.py`

## Troubleshooting Common Issues

### Gazebo Fails to Start
- Check ROS 2 environment sourcing
- Verify GPU drivers and graphics support
- Ensure Gazebo packages are installed

### Unity-ROS Connection Fails
- Confirm both systems are on the same network
- Check firewall settings for ROS communication
- Verify ROS bridge is running

### Sensor Data Not Generating
- Check that sensor plugins are loaded in Gazebo
- Verify the sensor simulation nodes are running
- Ensure proper TF transforms are published

## Next Steps

After completing this quickstart:

1. Dive deeper into each chapter to master specific aspects
2. Experiment with custom humanoid robot models
3. Develop your own perception algorithms using the generated sensor data
4. Explore advanced topics like multi-robot simulation and complex environments

## Support Resources

- **Documentation**: Full reference in the Digital Twin module
- **Community**: Check the project's GitHub discussions
- **Issues**: Report problems on the GitHub issue tracker
- **Tutorials**: Look for step-by-step tutorials in the examples directory