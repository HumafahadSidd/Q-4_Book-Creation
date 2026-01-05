---
sidebar_position: 6
---

# Chapter 5: Robot Description with URDF

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Understand the structure and components of URDF files
- Create basic robot models using URDF
- Define joints, links, and physical properties
- Model a simple humanoid robot structure
- Visualize URDF models in RViz

## 5.1 Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot, including:
- Kinematic structure (links and joints)
- Visual and collision properties
- Inertial properties
- Sensors and actuators

URDF is essential for simulation, visualization, and understanding robot kinematics.

## 5.2 URDF Structure

A basic URDF file contains:
- Links: Rigid parts of the robot (e.g., body, arms, legs)
- Joints: Connections between links (e.g., revolute, prismatic, fixed)
- Materials: Visual appearance properties
- Transmissions: Mapping between joints and actuators

## 5.3 Links

Links represent rigid bodies in the robot. Each link has:
- Visual: How the link appears in simulation
- Collision: How the link interacts with other objects
- Inertial: Mass, center of mass, and inertia properties

Example link definition:
```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

## 5.4 Joints

Joints connect links and define their relative motion. Common joint types:
- Fixed: No motion between links
- Revolute: Rotational motion around an axis
- Continuous: Revolute joint without limits
- Prismatic: Linear motion along an axis

Example joint definition:
```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.2 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

## 5.5 Creating a Simple Humanoid Model

A basic humanoid model includes:
- Torso (trunk)
- Head
- Arms (upper arm, lower arm, hand)
- Legs (upper leg, lower leg, foot)

Example simplified humanoid torso:
```xml
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>
</robot>
```

## 5.6 Visualizing URDF Models

URDF models can be visualized using RViz or other tools:
- Use the RobotModel plugin in RViz
- Check joint limits and ranges
- Verify collision and visual properties

## 5.7 Chapter Summary

In this chapter, we learned about URDF and how to describe robots using XML. We covered links, joints, and how to create a basic humanoid model. URDF is fundamental for simulation and visualization of robotic systems.

In the next chapter, we'll explore Gazebo, the physics simulator for robotics.

## Exercises

1. Create a URDF model of a simple robot with at least 3 links and 2 joints.
2. Add visual and collision properties to your robot model.
3. Visualize your URDF model in RViz.

---