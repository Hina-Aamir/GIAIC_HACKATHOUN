# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

## Prerequisites

Before starting this module, you should have:

1. Completed Module 1 (ROS 2 fundamentals)
2. Basic understanding of humanoid robot kinematics
3. Linux environment (Ubuntu 22.04 recommended) with ROS 2 Humble installed
4. Development environment with Unity 2022.3 LTS

## Setting Up the Environment

### Installing Gazebo

```bash
# Install Gazebo Fortress (or Harmonic)
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install additional simulation tools
sudo apt install ros-humble-ros-gz
```

### Setting Up Unity Environment

```bash
# Download and install Unity Hub
# Install Unity 2022.3 LTS with the following modules:
# - Linux Build Support (IL2CPP)
# - Universal Windows Platform Build Support
```

### Installing Unity ROS 2 Package

1. Open Unity Hub and create a new 3D project
2. Go to Window → Package Manager
3. Install "ROS 2 for Unity" package from the Unity Asset Store
4. Configure the package for your ROS 2 distribution

## Chapter 1: Physics Simulation in Gazebo

### Creating Your First Gazebo Simulation

1. Create a simple humanoid robot model:
```bash
mkdir -p ~/simulation_ws/src/humanoid_model
cd ~/simulation_ws/src/humanoid_model
```

2. Create a basic URDF file for a simple humanoid:
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.4" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

3. Create a simple world file for Gazebo:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

4. Launch the simulation:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo with your world
gazebo --verbose path/to/your/world.world
```

### Understanding Physics Properties

1. Gravity effects: Observe how objects fall with realistic acceleration
2. Collision detection: Watch how objects interact when they come into contact
3. Friction: Notice how objects slide or grip based on surface properties
4. Mass and inertia: See how these properties affect movement and stability

## Chapter 2: High-Fidelity Visualization in Unity

### Setting Up Unity Scene

1. Create a new Unity 3D project
2. Import the humanoid robot model (as FBX or other supported format)
3. Configure materials and lighting:
   - Add a Directional Light to simulate sun
   - Create PBR materials with realistic albedo, normal, and roughness maps
   - Set up reflection probes for accurate reflections

### Basic Unity-ROS 2 Integration

1. Create a simple ROS 2 publisher in Unity:
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityROSPublisher : MonoBehaviour
{
    ROSConnection ros;
    float publishRate = 1f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        InvokeRepeating("PublishClock", 0, publishRate);
    }

    void PublishClock()
    {
        ros.Send("unity_clock", new TimeMsg());
    }
}
```

2. Test the connection between Unity and ROS 2:
```bash
# In a terminal, check if the topic is available
source /opt/ros/humble/setup.bash
ros2 topic list | grep unity
```

## Chapter 3: Sensor Simulation

### LiDAR Simulation in Gazebo

1. Add a LiDAR sensor to your URDF:
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

2. Visualize LiDAR data:
```bash
# Subscribe to LiDAR data
source /opt/ros/humble/setup.bash
ros2 topic echo /lidar/scan sensor_msgs/msg/LaserScan
```

### Depth Camera Simulation

1. Add a depth camera to your robot:
```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/rgb/image_raw:=rgb/image_raw</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Simulation

1. Add an IMU sensor to your robot:
```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

## Integration Exercise

Combine all components by:
1. Creating a humanoid robot with all three sensor types
2. Running physics simulation in Gazebo
3. Visualizing the same scene in Unity
4. Subscribing to all sensor data streams
5. Validating that simulated data matches expected ranges

## Troubleshooting

- If Gazebo doesn't start, check if ROS 2 environment is sourced
- If Unity-ROS 2 connection fails, verify IP addresses and ports
- If sensor data seems unrealistic, check noise parameters and physics properties
- If performance is poor, reduce simulation complexity or adjust quality settings

## Next Steps

After completing this quickstart:
1. Explore advanced physics properties in Gazebo
2. Implement more complex visualization techniques in Unity
3. Experiment with different sensor configurations
4. Integrate with AI agents from Module 1 for autonomous control