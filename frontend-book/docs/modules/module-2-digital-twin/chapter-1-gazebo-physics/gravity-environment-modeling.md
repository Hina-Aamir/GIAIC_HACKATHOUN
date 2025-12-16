---
sidebar_position: 4
title: "Gravity and Environment Modeling"
---

# Gravity and Environment Modeling

## Introduction

Gravity and environment modeling are fundamental aspects of creating realistic humanoid robot simulations in Gazebo. Properly configured gravity ensures that robots behave naturally, while accurate environment modeling provides the context for realistic interactions. This section covers how to create and configure realistic environments for humanoid robot digital twins.

## Gravity Configuration

### Global Gravity Settings

Gravity is defined globally for each simulation world and affects all objects within that world. The default Earth gravity is typically set to 9.81 m/s² in the negative Z direction (downward):

```xml
<world name="humanoid_world">
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>
    <!-- ... other physics settings ... -->
  </physics>
  <!-- ... world content ... -->
</world>
```

### Custom Gravity Scenarios

For different scenarios, you can adjust gravity values:

- **Reduced gravity**: Simulate environments like the moon (1.62 m/s²)
- **Zero gravity**: Simulate space environments
- **Negative gravity**: For testing special scenarios
- **Directional gravity**: For simulating robots on slopes or in special environments

```xml
<!-- Moon gravity simulation -->
<gravity>0 0 -1.62</gravity>

<!-- Zero gravity for space simulation -->
<gravity>0 0 0</gravity>

<!-- Gravity in a different direction (e.g., robot on a wall) -->
<gravity>-9.81 0 0</gravity>
```

### Gravity Considerations for Humanoid Robots

When configuring gravity for humanoid robots, consider:

- **Balance algorithms**: Ensure that your robot's balance controller can handle the gravity setting
- **Walking patterns**: Gravity affects how the robot's gait behaves
- **Stability**: Higher gravity requires more robust stabilization systems
- **Energy consumption**: Gravity affects the power requirements for maintaining posture

## Environment Modeling

### Terrain and Ground Surfaces

Creating realistic ground surfaces is essential for humanoid robot simulation:

#### Flat Ground Plane
The most common environment is a flat ground plane:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

#### Complex Terrain
For more complex environments, you can create custom terrain:

```xml
<model name="complex_terrain">
  <link name="terrain_link">
    <collision name="terrain_collision">
      <geometry>
        <heightmap>
          <uri>model://terrain/materials/textures/heightmap.png</uri>
          <size>10 10 2</size>  <!-- width, depth, height -->
        </heightmap>
      </geometry>
    </collision>
    <visual name="terrain_visual">
      <geometry>
        <heightmap>
          <uri>model://terrain/materials/textures/heightmap.png</uri>
          <size>10 10 2</size>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

### Obstacle Modeling

Creating obstacles helps test robot navigation and interaction capabilities:

```xml
<model name="obstacle_box">
  <pose>2 0 0.5 0 0 0</pose>  <!-- Position and orientation -->
  <link name="box_link">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <collision name="box_collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="box_visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Environmental Objects

Create various environmental objects that humanoid robots might encounter:

#### Walls and Barriers
```xml
<model name="wall">
  <pose>0 5 1 0 0 0</pose>
  <link name="wall_link">
    <collision name="wall_collision">
      <geometry>
        <box>
          <size>10 0.2 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="wall_visual">
      <geometry>
        <box>
          <size>10 0.2 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.3 0.3 0.3 1</ambient>
        <diffuse>0.5 0.5 0.5 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

#### Steps and Stairs
```xml
<model name="steps">
  <pose>5 0 0 0 0 0</pose>
  <link name="step1">
    <collision name="step1_collision">
      <geometry>
        <box>
          <size>2 1 0.2</size>
        </box>
      </geometry>
    </collision>
    <visual name="step1_visual">
      <geometry>
        <box>
          <size>2 1 0.2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.4 0.4 0.4 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
  </link>
  <link name="step2">
    <pose>0 0 0.2 0 0 0</pose>
    <collision name="step2_collision">
      <geometry>
        <box>
          <size>2 1 0.2</size>
        </box>
      </geometry>
    </collision>
    <visual name="step2_visual">
      <geometry>
        <box>
          <size>2 1 0.2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.4 0.4 0.4 1</ambient>
        <diffuse>0.6 0.6 0.6 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## Environmental Physics Properties

### Surface Properties

Configure surface properties to affect how the humanoid robot interacts with different surfaces:

```xml
<world name="humanoid_world">
  <!-- Physics engine configuration with surface properties -->
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>
    <ode>
      <solver>
        <type>quick</type>
        <iters>10</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>

  <!-- Ground plane with specific surface properties -->
  <include>
    <uri>model://ground_plane</uri>
    <pose>0 0 0 0 0 0</pose>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>  <!-- Static friction coefficient -->
          <mu2>0.8</mu2>  <!-- Secondary friction coefficient -->
        </ode>
      </friction>
    </surface>
  </include>
</world>
```

### Friction Considerations

Different surfaces require different friction coefficients for realistic humanoid robot interaction:

- **High friction surfaces** (μ = 0.8-1.0): Concrete, rubber, carpet - good for stable walking
- **Medium friction surfaces** (μ = 0.4-0.7): Wood, tile - normal walking conditions
- **Low friction surfaces** (μ = 0.1-0.3): Ice, wet surfaces - challenging for balance

## Lighting and Visual Environment

### Sun Configuration

Configure the sun for realistic lighting in the environment:

```xml
<model name="sun">
  <static>true</static>
  <link name="sun_link">
    <light name="sun_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </link>
</model>
```

### Ambient Light

Configure ambient light for the entire environment:

```xml
<world name="humanoid_world">
  <ambient>0.4 0.4 0.4 1</ambient>  <!-- RGBA values -->
  <!-- ... other world properties ... -->
</world>
```

## Practical Example: Complete Environment for Humanoid Robot

Here's a complete example of an environment configured for humanoid robot simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Physics configuration -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Ambient light -->
    <ambient>0.3 0.3 0.3 1</ambient>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
      </surface>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Example obstacle -->
    <model name="obstacle">
      <pose>3 0 0.5 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="obstacle_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="obstacle_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>1 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Validation and Testing

### Environment Validation

Before running complex humanoid robot simulations:

1. **Check gravity settings**: Ensure gravity values are appropriate for your use case
2. **Test basic interactions**: Verify that the robot interacts properly with surfaces
3. **Validate obstacles**: Ensure obstacles have appropriate collision properties
4. **Performance testing**: Check that the environment doesn't cause performance issues

### Physics Accuracy

Validate that your environment modeling is physically accurate:

- Gravity should match real-world expectations
- Friction coefficients should reflect real surface properties
- Collision detection should work properly with all objects
- Environmental objects should have appropriate mass and inertia

## Advanced Environment Features

### Dynamic Environments

For more complex simulations, you can create dynamic environments:

- Moving platforms
- Variable terrain
- Interactive objects
- Weather effects (wind, etc.)

### Multi-Environment Simulation

Consider creating multiple environments for different testing scenarios:

- Indoor environments for navigation
- Outdoor environments with terrain variations
- Obstacle courses for mobility testing
- Specialized environments for specific tasks

## Summary

Proper gravity and environment modeling is essential for creating realistic humanoid robot simulations in Gazebo. By carefully configuring these elements, you can create digital twins that accurately reflect the behavior of real humanoid robots in various environments. This completes Chapter 1 on Physics Simulation in Gazebo.