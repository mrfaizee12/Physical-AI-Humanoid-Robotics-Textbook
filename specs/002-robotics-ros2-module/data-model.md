# Data Model for ROS 2 Basics

This document defines the key conceptual entities for the "Robotic Nervous System (ROS 2)" module. Unlike a traditional software data model with database tables, this model defines the core concepts being taught.

## Core Entities

### 1. ROS 2 Node

- **Description**: A `Node` is the fundamental unit of execution in ROS 2. It is a process that performs computation and communicates with other nodes. Think of it as a small, single-purpose program within the larger robotics system.
- **Key Attributes**:
    - **Name**: A unique identifier for the node within the ROS 2 graph (e.g., `camera_publisher`, `motor_controller`).
    - **Publishers**: A list of topics the node sends messages to.
    - **Subscribers**: A list of topics the node receives messages from.
    - **Services**: A list of services the node provides (for request/response interactions).
    - **Clients**: A list of services the node uses.
- **Relationships**:
    - A Node can have zero or more Publishers, Subscribers, Services, and Clients.

### 2. ROS 2 Topic

- **Description**: A `Topic` is a named bus for messages. It acts as a channel for nodes to exchange data without being directly aware of each other. This is the primary method for continuous data streams (e.g., sensor readings, motor commands).
- **Key Attributes**:
    - **Name**: A unique identifier for the topic (e.g., `/cmd_vel`, `/scan`, `/image_raw`).
    - **Message Type**: A strict data structure defining the content of messages on that topic (e.g., `geometry_msgs/Twist`, `sensor_msgs/LaserScan`).
- **Relationships**:
    - A Topic has one-to-many relationship with Nodes (one node publishes, many can subscribe, or vice-versa).

### 3. ROS 2 Service

- **Description**: A `Service` provides a request/response model of communication. It's used for remote procedure calls (RPCs) where a node (the client) needs a direct response from another node (the server) before continuing. This is used for discrete actions, not continuous data streams.
- **Key Attributes**:
    - **Name**: A unique identifier for the service (e.g., `/spawn`, `/reset_odometry`).
    - **Service Type**: A strict data structure defining the request and response message formats.
- **Relationships**:
    - A Service is provided by one Node (the server) and can be called by one or more other Nodes (clients).

### 4. URDF (Unified Robot Description Format)

- **Description**: A URDF is an XML file that represents the physical structure of a robot. It defines the robot's parts (links) and how they are connected (joints). It is essential for simulation, visualization, and many planning algorithms.
- **Key Sub-Entities**:
    - **Link**: A rigid part of the robot's body. It has inertial, visual, and collision properties.
    - **Joint**: Connects two links. It defines the type of motion allowed between them (e.g., revolute, prismatic, fixed) and sets movement limits.
- **Relationships**:
    - A robot model consists of a tree of `Links` connected by `Joints`.

### State Transitions

- The state of the ROS 2 system (the "graph") changes dynamically:
    - A `Node` can be in a `running` or `stopped` state.
    - The set of available `Topics` and `Services` changes as nodes are started and stopped.
    - The state of a `Joint` (its position/angle) changes over time based on commands published to topics.
