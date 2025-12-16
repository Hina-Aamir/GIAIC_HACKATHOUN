# URDF Structure and Components

This section delves into the detailed structure of URDF (Unified Robot Description Format) files and their components, focusing on how they apply to humanoid robot modeling.

## URDF XML Structure

URDF is an XML-based format with a hierarchical structure. The root element is always `<robot>`, which contains links, joints, and other elements:

```xml
<?xml version="1.0"?>
<robot name="robot_name" version="1.0">
  <!-- Links -->
  <link name="link_name">
    <!-- Visual properties -->
    <visual>
      <!-- Visual geometry and material -->
    </visual>

    <!-- Collision properties -->
    <collision>
      <!-- Collision geometry -->
    </collision>

    <!-- Inertial properties -->
    <inertial>
      <!-- Mass and inertia properties -->
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <!-- Joint properties -->
  </joint>

  <!-- Materials -->
  <material name="material_name">
    <!-- Material properties -->
  </material>

  <!-- Transmissions -->
  <transmission name="transmission_name">
    <!-- Actuator definitions -->
  </transmission>
</robot>
```

## Detailed Link Structure

Each link represents a rigid body and can contain multiple sub-elements:

### Visual Element

The visual element defines how a link appears in visualization tools:

```xml
<visual>
  <!-- Origin: position and orientation relative to link frame -->
  <origin xyz="0.1 0.0 0.05" rpy="0 0 0"/>

  <!-- Geometry: shape of the visual representation -->
  <geometry>
    <!-- Box geometry -->
    <box size="0.2 0.1 0.1"/>

    <!-- Or cylinder geometry -->
    <!-- <cylinder radius="0.05" length="0.2"/> -->

    <!-- Or sphere geometry -->
    <!-- <sphere radius="0.05"/> -->

    <!-- Or mesh geometry -->
    <!-- <mesh filename="package://robot_description/meshes/link.stl" scale="1 1 1"/> -->
  </geometry>

  <!-- Material: color and appearance -->
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
    <!-- Or reference a material defined elsewhere -->
    <!-- <name>red_material</name> -->
  </material>
</visual>
```

### Collision Element

The collision element defines the geometry used for collision detection:

```xml
<collision>
  <!-- Origin: same as visual element -->
  <origin xyz="0.1 0.0 0.05" rpy="0 0 0"/>

  <!-- Geometry: shape for collision detection -->
  <geometry>
    <!-- Often similar to visual geometry but can be simplified -->
    <box size="0.2 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Element

The inertial element defines the physical properties for dynamics simulation:

```xml
<inertial>
  <!-- Origin: center of mass offset -->
  <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>

  <!-- Mass: in kilograms -->
  <mass value="0.5"/>

  <!-- Inertia matrix: 6 independent values for 3D rotation -->
  <inertia
    ixx="0.001" ixy="0.0" ixz="0.0"
    iyy="0.002" iyz="0.0"
    izz="0.001"/>
</inertial>
```

## Joint Structure and Types

Joints define how links connect and move relative to each other:

### Joint Attributes

```xml
<joint name="joint_name" type="joint_type">
  <!-- Joint type determines the degrees of freedom -->
  <!-- Common types: revolute, continuous, prismatic, fixed, floating, planar -->
</joint>
```

### Joint Sub-elements

```xml
<joint name="example_joint" type="revolute">
  <!-- Parent link: the link that is the origin of the joint -->
  <parent link="upper_body"/>

  <!-- Child link: the link that moves relative to the parent -->
  <child link="lower_body"/>

  <!-- Origin: position and orientation of the joint in the parent frame -->
  <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>

  <!-- Axis: direction of the joint motion in the joint frame -->
  <axis xyz="0 1 0"/>

  <!-- Limits: for joints with limited range of motion -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>

  <!-- Dynamics: friction and damping parameters -->
  <dynamics damping="0.1" friction="0.01"/>

  <!-- Safety controller: limits for safety -->
  <safety_controller k_position="10" k_velocity="10" soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
</joint>
```

### Joint Types for Humanoid Robots

Different joint types are used for different parts of a humanoid robot:

#### Revolute Joints
For joints with limited rotation:

```xml
<joint name="knee_joint" type="revolute">
  <parent link="thigh"/>
  <child link="shin"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="200" velocity="5"/>
</joint>
```

#### Continuous Joints
For joints that can rotate indefinitely:

```xml
<joint name="wrist_joint" type="continuous">
  <parent link="forearm"/>
  <child link="hand"/>
  <axis xyz="0 0 1"/>
  <limit effort="50" velocity="10"/>
</joint>
```

#### Fixed Joints
For parts that don't move relative to each other:

```xml
<joint name="sensor_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

## Advanced URDF Elements

### Materials

Materials define the visual appearance and can be reused across multiple links:

```xml
<material name="robot_gray">
  <color rgba="0.5 0.5 0.5 1.0"/>
  <!-- Optional: texture -->
  <!-- <texture filename="path/to/texture.png"/> -->
</material>

<material name="red">
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>
```

### Transmissions

Transmissions define how actuators connect to joints:

```xml
<transmission name="joint1_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="joint1_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo Extensions

Gazebo-specific elements can be included within URDF:

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <self_collide>true</self_collide>
</gazebo>

<!-- Gazebo plugin -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/robot</robotNamespace>
  </plugin>
</gazebo>
```

## URDF for Humanoid Robot Components

### Torso Structure

```xml
<!-- Torso link -->
<link name="torso">
  <visual>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <capsule length="0.4" radius="0.1"/>
    </geometry>
    <material name="torso_material">
      <color rgba="0.8 0.8 0.8 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <geometry>
      <capsule length="0.4" radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
  </inertial>
</link>
```

### Limb Structure

```xml
<!-- Example leg structure -->
<link name="upper_leg">
  <visual>
    <geometry>
      <capsule length="0.4" radius="0.08"/>
    </geometry>
    <material name="limb_material">
      <color rgba="0.6 0.6 0.6 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.4" radius="0.08"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="3.0"/>
    <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
  </inertial>
</link>

<joint name="hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_leg"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="300" velocity="3"/>
</joint>
```

## URDF Validation and Tools

### Validation Commands

```bash
# Validate URDF syntax
check_urdf robot.urdf

# Generate kinematic chain graph
urdf_to_graphiz robot.urdf

# Visualize in RViz
ros2 run rviz2 rviz2
```

### Common Validation Issues

1. **Missing parent/child links**: Ensure all joint references exist
2. **Disconnected links**: All links should be connected through joints
3. **Inconsistent units**: Use consistent units throughout
4. **Invalid inertia matrices**: Ensure inertia values are physically plausible

## Using xacro for Complex URDF

For complex humanoid robots, xacro (XML Macros) can simplify URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_mass" value="10.0" />
  <xacro:property name="limb_radius" value="0.08" />

  <!-- Macro for symmetric limbs -->
  <xacro:macro name="limb" params="side parent_link position">
    <link name="${side}_upper_limb">
      <visual>
        <geometry>
          <capsule length="0.4" radius="${limb_radius}"/>
        </geometry>
      </visual>
      <inertial>
        <mass value="3.0"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${side}_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_limb"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-2.0" upper="2.0" effort="200" velocity="4"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:limb side="left" parent_link="torso" position="0 0.1 -0.1"/>
  <xacro:limb side="right" parent_link="torso" position="0 -0.1 -0.1"/>

</robot>
```

## Summary

URDF structure provides a comprehensive way to describe humanoid robots with all necessary geometric, kinematic, and dynamic properties. Understanding the hierarchy of elements, proper joint definitions, and best practices for organization is crucial for creating effective robot models. The use of xacro can significantly simplify complex humanoid robot descriptions by enabling parameterization and reusability.