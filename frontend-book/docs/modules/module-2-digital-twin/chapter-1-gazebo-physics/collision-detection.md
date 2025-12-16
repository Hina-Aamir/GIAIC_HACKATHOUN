---
sidebar_position: 3
title: "Collision Detection in Gazebo"
---

# Collision Detection in Gazebo

## Introduction

Collision detection is a critical component of physics simulation for humanoid robots in Gazebo. It ensures that robot parts interact realistically with each other and the environment, preventing limbs from passing through each other or walking through obstacles. Proper collision detection is essential for stable and realistic humanoid robot simulation.

## Types of Collision Detection

### Self-Collision Detection

Self-collision detection prevents different parts of the humanoid robot from intersecting with each other. This is particularly important for complex humanoid robots with multiple limbs that move in close proximity.

For humanoid robots, self-collision typically occurs between:
- Arms and torso
- Legs during walking or complex movements
- Hands and other body parts during manipulation tasks

### Environment Collision

Environment collision detection prevents the humanoid robot from passing through environmental objects like walls, floors, and obstacles. This ensures realistic interaction with the simulated world.

### Contact Physics

Contact physics determines how forces are applied when collisions occur, affecting how the robot responds to impacts and interactions.

## Collision Shapes in Gazebo

Gazebo supports various collision shapes for humanoid robot simulation:

### Primitive Shapes

**Box**: Rectangular collision shapes suitable for simple body parts like torso sections or basic limbs.

```xml
<collision name="torso_collision">
  <geometry>
    <box>
      <size>0.3 0.2 0.5</size>
    </box>
  </geometry>
</collision>
```

**Sphere**: Perfect for rounded joints or simple body parts.

```xml
<collision name="head_collision">
  <geometry>
    <sphere>
      <radius>0.1</radius>
    </sphere>
  </geometry>
</collision>
```

**Cylinder**: Good for limbs like arms and legs.

```xml
<collision name="arm_collision">
  <geometry>
    <cylinder>
      <length>0.4</length>
      <radius>0.05</radius>
    </cylinder>
  </geometry>
</collision>
```

**Capsule**: Ideal for humanoid limbs as it combines cylinder and spherical caps, providing better collision detection than simple cylinders.

```xml
<collision name="leg_collision">
  <geometry>
    <capsule>
      <length>0.4</length>
      <radius>0.05</radius>
    </capsule>
  </geometry>
</collision>
```

### Mesh Shapes

For complex humanoid robot parts, you can use mesh-based collision shapes:

```xml
<collision name="foot_collision">
  <geometry>
    <mesh>
      <uri>model://humanoid_robot/meshes/foot_collision.dae</uri>
    </mesh>
  </geometry>
</collision>
```

## Collision Detection Algorithms

Gazebo uses multiple collision detection algorithms:

### FCL (Flexible Collision Library)
- Supports both primitive and mesh shapes
- Good performance for humanoid robot simulation
- Handles complex collision scenarios effectively

### Bullet Collision
- Part of the Bullet Physics engine
- Excellent for complex humanoid kinematics
- Good integration with Bullet physics engine

## Configuration Parameters

### Collision Properties

Collision properties define how collisions behave in the simulation:

```xml
<collision name="collision_link">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>  <!-- Static friction coefficient -->
        <mu2>1.0</mu2>  <!-- Secondary friction coefficient -->
        <slip1>0.0</slip1>  <!-- First slip coefficient -->
        <slip2>0.0</slip2>  <!-- Second slip coefficient -->
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.01</restitution_coefficient>  <!-- Bounciness -->
      <threshold>100000.0</threshold>  <!-- Velocity threshold for bouncing -->
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0.0</soft_cfm>  <!-- Soft constraint force mixing -->
        <soft_erp>0.2</soft_erp>  <!-- Soft error reduction parameter -->
        <kp>1e+13</kp>  <!-- Contact stiffness -->
        <kd>1.0</kd>  <!-- Contact damping -->
        <max_vel>100.0</max_vel>  <!-- Maximum contact correction velocity -->
        <min_depth>0.0</min_depth>  <!-- Minimum contact depth -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Self-Collision Configuration

To enable or disable self-collision for specific joints:

```xml
<link name="arm_link">
  <self_collide>false</self_collide>  <!-- Disable self-collision for this link -->
  <!-- ... other link properties ... -->
</link>
```

## Best Practices for Humanoid Robot Collision Detection

### Shape Selection
- Use capsules for limbs (arms, legs) as they provide good collision detection with reasonable performance
- Use boxes for torso sections and simple body parts
- Use spheres for joints and rounded components
- Use simplified meshes for complex parts to maintain performance

### Performance Optimization
- Use simpler collision shapes where possible
- Reduce the number of collision elements for better performance
- Use collision groups to limit unnecessary collision checks
- Adjust physics update rates based on complexity

### Accuracy vs Performance
- Balance collision accuracy with simulation performance
- Use more detailed collision shapes for critical interactions
- Simplify collision shapes for parts that rarely interact

## Practical Example: Collision Configuration for Humanoid Robot

Here's a complete example of collision configuration for a humanoid robot's leg:

```xml
<link name="lower_leg">
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
  </inertial>

  <visual name="lower_leg_visual">
    <geometry>
      <capsule>
        <length>0.4</length>
        <radius>0.05</radius>
      </capsule>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 0.8"/>
    </material>
  </visual>

  <collision name="lower_leg_collision">
    <geometry>
      <capsule>
        <length>0.4</length>
        <radius>0.05</radius>
      </capsule>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>
          <mu2>0.8</mu2>
        </ode>
      </friction>
      <contact>
        <ode>
          <soft_erp>0.2</soft_erp>
          <soft_cfm>0.0</soft_cfm>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

## Troubleshooting Common Issues

### Excessive Collisions
- Check joint limits and ensure they're properly configured
- Verify that collision shapes don't overlap in the default pose
- Adjust contact parameters to reduce jitter

### Penetration Issues
- Increase physics solver iterations
- Adjust contact surface layer parameters
- Verify that collision shapes are properly sized

### Performance Problems
- Simplify collision shapes where possible
- Reduce the number of collision elements
- Adjust physics update rates appropriately

## Validation Techniques

### Visual Inspection
- Use Gazebo's collision visualization to verify collision shapes
- Check for overlaps in the default robot pose
- Validate collision shapes during movement

### Simulation Testing
- Test basic movements to ensure no unwanted collisions
- Verify that the robot can move through expected ranges of motion
- Check for stability during dynamic movements

## Summary

Proper collision detection is essential for realistic humanoid robot simulation in Gazebo. By carefully selecting collision shapes and configuring parameters, you can achieve both accuracy and performance in your digital twin simulation. The next section will cover gravity and environment modeling.