# Humanoid Models in URDF

This section explores how to create and structure URDF models specifically for humanoid robots, covering the anatomical considerations, kinematic chains, and practical examples of humanoid robot models.

## Humanoid Robot Anatomy

Humanoid robots are designed to mimic human anatomy and movement patterns. When modeling humanoid robots in URDF, it's important to consider the following anatomical components:

### Major Body Parts

1. **Torso/Pelvis**: The central core of the robot
2. **Head**: Contains sensors and provides orientation
3. **Arms**: Shoulders, upper arms, forearms, and hands
4. **Legs**: Hips, thighs, shins, and feet

### Degrees of Freedom (DOF)

Humanoid robots typically have a high number of DOF to achieve human-like movement:

- **Torso**: 3-6 DOF (depending on complexity)
- **Each arm**: 6-8 DOF (shoulder: 3, elbow: 1, wrist: 2-3)
- **Each leg**: 6-7 DOF (hip: 3, knee: 1, ankle: 2)
- **Head**: 2-3 DOF (neck rotation, tilt)

## Complete Humanoid Robot URDF Example

Here's a comprehensive example of a simplified humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="torso_mass" value="15.0"/>
  <xacro:property name="limb_mass" value="5.0"/>
  <xacro:property name="head_mass" value="2.0"/>

  <!-- Base/Fixed link -->
  <link name="base_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Pelvis (Root of the robot) -->
  <joint name="base_to_pelvis" type="fixed">
    <parent link="base_link"/>
    <child link="pelvis"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="pelvis">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.25 0.1"/>
      </geometry>
      <material name="robot_gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.25 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="revolute">
    <parent link="pelvis"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="300" velocity="2"/>
  </joint>

  <link name="torso">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.1"/>
      </geometry>
      <material name="robot_torso">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="50" velocity="3"/>
  </joint>

  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="robot_head">
        <color rgba="0.8 0.8 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 0.15 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="4"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <capsule length="0.3" radius="0.06"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.3" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI}" effort="80" velocity="5"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>

  <!-- Right Arm (symmetric) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.05 -0.15 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="4"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <capsule length="0.3" radius="0.06"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.3" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI}" effort="80" velocity="5"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.003"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="pelvis"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.05 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="200" velocity="3"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.08"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.4" radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI}" effort="180" velocity="4"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.07"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.4" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="100" velocity="3"/>
  </joint>

  <link name="left_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="robot_foot">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Leg (symmetric) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="pelvis"/>
    <child link="right_thigh"/>
    <origin xyz="0 -0.05 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="200" velocity="3"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.08"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.4" radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="${M_PI}" effort="180" velocity="4"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.07"/>
      </geometry>
      <material name="robot_limb">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.4" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="100" velocity="3"/>
  </joint>

  <link name="right_foot">
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="robot_foot">
        <color rgba="0.4 0.4 0.4 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Materials -->
  <material name="robot_gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="robot_torso">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <material name="robot_head">
    <color rgba="0.8 0.8 0.8 1.0"/>
  </material>
  <material name="robot_limb">
    <color rgba="0.6 0.6 0.6 1.0"/>
  </material>
  <material name="robot_foot">
    <color rgba="0.4 0.4 0.4 1.0"/>
  </material>

</robot>
```

## Kinematic Chains in Humanoid Robots

Humanoid robots have multiple kinematic chains that work together:

### Forward Kinematic Chains

1. **Left Arm Chain**: torso → left_shoulder → left_upper_arm → left_lower_arm
2. **Right Arm Chain**: torso → right_shoulder → right_upper_arm → right_lower_arm
3. **Left Leg Chain**: pelvis → left_hip → left_thigh → left_shin → left_foot
4. **Right Leg Chain**: pelvis → right_hip → right_thigh → right_shin → right_foot

### Closed Kinematic Chains

When both feet are on the ground, the robot forms closed kinematic chains that affect the dynamics and control.

## Common Humanoid Joint Configurations

### Hip Joint Configuration
```xml
<!-- 3 DOF hip joint -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="pelvis"/>
  <child link="left_hip_roll_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="200" velocity="3"/>
</joint>

<joint name="left_hip_roll" type="revolute">
  <parent link="left_hip_roll_link"/>
  <child link="left_thigh"/>
  <axis xyz="1 0 0"/>
  <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="250" velocity="2"/>
</joint>
```

### Shoulder Joint Configuration
```xml
<!-- 3 DOF shoulder joint -->
<joint name="left_shoulder_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_shoulder_pitch_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="4"/>
</joint>

<joint name="left_shoulder_pitch" type="revolute">
  <parent link="left_shoulder_pitch_link"/>
  <child link="left_upper_arm"/>
  <axis xyz="0 1 0"/>
  <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="120" velocity="4"/>
</joint>
```

## Sensor Integration in Humanoid Models

Humanoid robots typically have various sensors that need to be modeled:

```xml
<!-- IMU in torso -->
<joint name="torso_imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<!-- Camera in head -->
<joint name="head_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<link name="camera_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

## Best Practices for Humanoid Modeling

### 1. Mass Distribution
- Ensure realistic mass properties for stable simulation
- Use appropriate inertial tensors
- Consider actuator masses in link definitions

### 2. Joint Limits
- Set realistic joint limits based on mechanical constraints
- Consider soft limits for safety
- Account for cable management and physical stops

### 3. Visualization vs Collision
- Use detailed meshes for visualization
- Use simplified geometries for collision detection
- Ensure collision geometry adequately represents the physical robot

### 4. Naming Conventions
- Use consistent naming (left_*, right_*)
- Follow ROS conventions
- Make names descriptive but concise

## Validation and Testing

### URDF Validation
```bash
# Check the URDF file
check_urdf simple_humanoid.urdf

# Generate kinematic graph
urdf_to_graphiz simple_humanoid.urdf
dot -Tpng simple_humanoid.gv -o simple_humanoid.png
```

### Visualization in RViz
```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2
# Add RobotModel display and set robot description to your URDF
```

### Simulation Testing
```bash
# Load into Gazebo for physics simulation
# Test joint limits and collision detection
# Verify kinematic behavior
```

## Summary

Creating accurate URDF models for humanoid robots requires careful consideration of human anatomy, kinematic chains, and physical properties. The examples provided demonstrate how to structure a complete humanoid model with proper mass properties, joint limits, and sensor integration. Following best practices for modeling ensures that the robot model will work effectively in simulation and provide a solid foundation for control algorithm development.