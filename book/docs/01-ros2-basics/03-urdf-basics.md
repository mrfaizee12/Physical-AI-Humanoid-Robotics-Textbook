---
sidebar_position: 3
sidebar_label: "3. URDF Basics"
---

# 3. URDF Basics: Describing a Robot's Structure

To simulate, control, or visualize a robot, ROS needs to understand its physical structure. The **Unified Robot Description Format (URDF)** is an XML-based standard for describing the geometry, kinematics, and dynamics of a robot.

A URDF file describes the robot as a tree of **links** connected by **joints**.

## Links: The Body Parts

A **link** is a rigid part of the robot's body. Each link has its own coordinate system and can have three types of properties defined:

1.  **Visual**: What the link looks like. This is usually defined with a simple geometric shape (like a box, cylinder, or sphere) or a 3D mesh file (like a `.stl` or `.dae`).
2.  **Collision**: What the physical boundary of the link is for collision detection. This is often a simplified version of the visual geometry to speed up physics calculations.
3.  **Inertial**: The link's mass and rotational inertia properties. This is crucial for accurate physics simulation.

Here is an example of a simple link definition:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.1" radius="0.2"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.1" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

## Joints: The Connections

A **joint** connects one link to another, defining the relationship and allowed motion between them. Every joint has a **parent** link and a **child** link. This is how the tree structure is formed.

Joints have several types, with the most common being:

-   **revolute**: A hinge joint that rotates around a single axis (e.g., an elbow or a wheel axle).
-   **continuous**: A revolute joint with no angle limits.
-   **prismatic**: A sliding joint that moves along a single axis.
-   **fixed**: A rigid connection between two links with no movement allowed.

Here is an example of a joint that connects a new `arm_link` to our `base_link`:

```xml
<joint name="base_to_arm" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.05"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```
This defines a revolute joint that allows the `arm_link` to rotate relative to the `base_link` around the Y-axis.

## A Complete URDF Example

Putting it all together, here is a complete, albeit simple, URDF file for a two-link arm.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0"/>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting Base to Arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

</robot>
```

This URDF file can be loaded by ROS 2 tools to visualize the robot model in RViz or simulate it in Gazebo. By publishing joint states, you can animate the model and bring your robot to life.
