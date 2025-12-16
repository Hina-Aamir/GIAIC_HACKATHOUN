# Chapter 3: Humanoid Robot Modeling

This chapter focuses on modeling humanoid robots using URDF (Unified Robot Description Format), which is essential for simulation, control, and understanding the physical structure of humanoid robots in ROS 2 systems.

## Introduction to Robot Modeling

Robot modeling is the process of creating a digital representation of a physical robot that captures its geometric, kinematic, and dynamic properties. For humanoid robots, this is particularly important because of their complex structure with multiple degrees of freedom that mimic human anatomy.

### Why Robot Modeling is Important

1. **Simulation**: Models allow testing of control algorithms in simulation before deployment
2. **Visualization**: Provides visual representation in tools like RViz
3. **Collision Detection**: Enables simulation of physical interactions
4. **Kinematics**: Allows computation of forward and inverse kinematics
5. **Dynamics**: Supports simulation of forces, torques, and motion

## URDF: Unified Robot Description Format

URDF is an XML-based format that describes robots. It defines the physical and visual properties of a robot in terms of links (rigid bodies) and joints (constraints between links).

### Core Concepts

- **Links**: Represent rigid bodies with visual, collision, and inertial properties
- **Joints**: Define how links connect and move relative to each other
- **Materials**: Define visual appearance of links
- **Transmissions**: Define how actuators connect to joints
- **Gazebo Extensions**: Additional tags for simulation in Gazebo

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define the rigid bodies -->
  <link name="base_link">
    <!-- Visual properties -->
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>

    <!-- Collision properties -->
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>

    <!-- Inertial properties -->
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="child_link">
    <!-- Child link definition -->
  </link>
</robot>
```

## Link Elements

Links represent rigid bodies in the robot. Each link can have multiple sub-elements:

### Visual Element
Defines how the link appears visually:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Can be box, cylinder, sphere, or mesh -->
    <box size="0.1 0.1 0.1"/>
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

### Collision Element
Defines the collision geometry for physics simulation:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Element
Defines mass properties for dynamics simulation:

```xml
<inertial>
  <mass value="0.1"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

## Joint Elements

Joints define the connection between links and their allowed motion. Common joint types include:

### Revolute Joint
Rotational joint with limited range:

```xml
<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="thigh"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotate about Y-axis -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Continuous Joint
Rotational joint without limits:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base"/>
  <child link="rotating_part"/>
  <axis xyz="0 0 1"/>
  <limit effort="10" velocity="10"/>
</joint>
```

### Fixed Joint
No movement between links:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0.1 0 0"/>
</joint>
```

## Humanoid Robot Anatomy in URDF

Humanoid robots have a specific anatomical structure that needs to be represented in URDF:

### Basic Humanoid Structure

```
base_link (pelvis)
├── torso
│   ├── head
│   ├── left_shoulder
│   │   ├── left_upper_arm
│   │   ├── left_lower_arm
│   │   └── left_hand
│   ├── right_shoulder
│   │   ├── right_upper_arm
│   │   ├── right_lower_arm
│   │   └── right_hand
│   ├── left_hip
│   │   ├── left_upper_leg
│   │   ├── left_lower_leg
│   │   └── left_foot
│   └── right_hip
│       ├── right_upper_leg
│       ├── right_lower_leg
│       └── right_foot
```

### Example Humanoid Link Definition

```xml
<!-- Torso -->
<link name="torso">
  <visual>
    <geometry>
      <capsule length="0.4" radius="0.1"/>
    </geometry>
    <material name="torso_color">
      <color rgba="0.8 0.8 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.4" radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

## URDF Best Practices

### File Organization
- Use separate files for different parts of the robot
- Use xacro for parameterization and macros
- Include proper naming conventions

### Naming Conventions
- Use descriptive names that reflect the robot anatomy
- Use consistent naming patterns (e.g., left_*, right_*)
- Follow ROS naming conventions (lowercase, underscores)

### Validation
- Always validate URDF files using tools
- Test in simulation before physical deployment
- Check for proper kinematic chains

## Tools for Working with URDF

### Command Line Tools
- `check_urdf <urdf_file>`: Validates URDF syntax
- `urdf_to_graphiz <urdf_file>`: Generates kinematic chain diagram

### Visualization
- RViz: Real-time visualization of robot model
- Gazebo: Physics simulation environment
- urdf_tutorial: Interactive URDF examples

## Summary

Robot modeling with URDF is fundamental to humanoid robotics. By properly defining links and joints that represent human anatomy, you create the foundation for simulation, control, and visualization of humanoid robots. The next sections will explore URDF structure in more detail and provide examples of humanoid robot models.