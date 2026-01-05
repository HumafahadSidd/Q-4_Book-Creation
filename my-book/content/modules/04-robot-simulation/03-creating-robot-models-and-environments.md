# Chapter 4.3: Creating Robot Models and Environments

## Learning Objectives

By the end of this chapter, students will be able to:
- Create detailed robot models using URDF and SDF formats
- Design realistic simulation environments with appropriate physics properties
- Configure sensors and actuators for robot models
- Implement custom Gazebo plugins for specialized behaviors
- Validate robot models for physical accuracy
- Design complex environments with multiple objects and interactions

## Introduction

Creating realistic robot models and environments is fundamental to effective simulation in robotics. A well-designed model accurately represents the physical properties, kinematics, and dynamics of the real robot, while a well-crafted environment provides appropriate challenges and scenarios for testing robotic algorithms. This chapter covers the essential techniques for creating robot models and environments that effectively bridge the gap between simulation and reality.

The quality of your robot models and environments directly impacts the validity of your simulation results and the effectiveness of sim-to-real transfer. Poorly designed models may lead to unrealistic behaviors that don't translate to real hardware, while well-designed models enable confident development and testing of robotic systems.

In this chapter, we'll explore both the URDF (Unified Robot Description Format) and SDF (Simulation Description Format) approaches to robot modeling, discuss environment design principles, and provide practical examples of creating complex simulation scenarios.

## Robot Modeling Fundamentals

### URDF vs SDF

Both URDF and SDF are XML-based formats for describing robots, but they serve different purposes:

**URDF (Unified Robot Description Format)**:
- Originally developed for ROS
- Focuses on robot kinematics and structure
- Widely used in ROS/ROS 2 ecosystems
- Defines links, joints, and basic visual/collision properties
- Requires additional Gazebo-specific tags for simulation properties

**SDF (Simulation Description Format)**:
- Developed for Gazebo and Ignition
- Comprehensive format for simulation environments
- Includes physics properties, sensors, plugins, and world descriptions
- Native format for Gazebo simulation
- More extensive than URDF for simulation-specific features

### Link and Joint Definitions

A robot model consists of links connected by joints. Links represent rigid bodies, while joints define the relationship between links.

**Basic URDF Structure:**
```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Additional links and joints -->
</robot>
```

### Inertial Properties

Accurate inertial properties are crucial for realistic dynamics simulation:

**Mass**: The mass of the link in kilograms
**Center of Mass**: The point where mass is concentrated
**Inertia Tensor**: Describes how mass is distributed relative to rotation axes

For common shapes, inertia values can be calculated:
- **Cylinder**: Ixx = Iyy = m*(3*r² + h²)/12, Izz = m*r²/2
- **Box**: Ixx = m*(h² + d²)/12, Iyy = m*(w² + d²)/12, Izz = m*(w² + h²)/12
- **Sphere**: Ixx = Iyy = Izz = 2*m*r²/5

## Creating Detailed Robot Models

### Multi-Link Robot Example

Let's create a more complex robot model with multiple links and joints:

**File: `~/gazebo_ws/src/robot_models/urdf/differential_drive_robot.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="differential_drive_robot">
  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.05" />
  <xacro:property name="wheel_width" value="0.02" />
  <xacro:property name="wheel_separation" value="0.3" />
  <xacro:property name="robot_length" value="0.3" />
  <xacro:property name="robot_width" value="0.2" />
  <xacro:property name="robot_height" value="0.1" />
  <xacro:property name="robot_mass" value="5.0" />
  
  <!-- Base chassis -->
  <link name="base_link">
    <inertial>
      <mass value="${robot_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia 
        ixx="${robot_mass*(robot_height*robot_height + robot_length*robot_length)/12}" 
        ixy="0" 
        ixz="0"
        iyy="${robot_mass*(robot_width*robot_width + robot_length*robot_length)/12}" 
        iyz="0"
        izz="${robot_mass*(robot_width*robot_width + robot_height*robot_height)/12}"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${robot_length} ${robot_width} ${robot_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${robot_length} ${robot_width} ${robot_height}"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Left wheel -->
  <link name="left_wheel">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia 
        ixx="0.001" ixy="0" ixz="0"
        iyy="0.001" iyz="0"
        izz="0.002"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Right wheel -->
  <link name="right_wheel">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia 
        ixx="0.001" ixy="0" ixz="0"
        iyy="0.001" iyz="0"
        izz="0.002"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="${-robot_length/2} ${-wheel_separation/2} 0" rpy="${-M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1"/>
  </joint>
  
  <!-- Right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="${-robot_length/2} ${wheel_separation/2} 0" rpy="${-M_PI/2} 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.1"/>
  </joint>
  
  <!-- Lidar mount -->
  <link name="lidar_mount">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.01"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.01"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint to attach lidar mount to base -->
  <joint name="lidar_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_mount"/>
    <origin xyz="${robot_length/3} 0 ${robot_height/2}" rpy="0 0 0"/>
  </joint>
  
  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>
  
  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>
  
  <!-- Differential drive plugin -->
  <gazebo>
    <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>${wheel_separation}</wheel_separation>
      <wheel_radius>${wheel_radius}</wheel_radius>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
    </plugin>
  </gazebo>
  
  <!-- Lidar sensor plugin -->
  <gazebo reference="lidar_mount">
    <sensor name="lidar" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
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
      <plugin filename="libignition-sensors-lidar-system.so" name="ignition::sensors::LidarSensor">
        <topic>scan</topic>
      </plugin>
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
</robot>
```

### Converting URDF to SDF

While URDF is more common in ROS environments, SDF is the native format for Gazebo. You can convert URDF to SDF using the `gz sdf` command:

```bash
# Convert URDF to SDF
gz sdf -p ~/gazebo_ws/src/robot_models/urdf/differential_drive_robot.urdf.xacro > ~/gazebo_ws/src/robot_models/models/differential_drive_robot/model.sdf
```

However, it's often better to create SDF models directly for Gazebo:

**File: `~/gazebo_ws/src/robot_models/models/differential_drive_robot/model.sdf`**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="differential_drive_robot">
    <!-- Chassis -->
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.054</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.063</iyy>
          <iyz>0</iyz>
          <izz>0.019</izz>
        </inertia>
      </inertial>
      
      <visual name="chassis_visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 1 0.8</ambient>
          <diffuse>0 0 1 0.8</diffuse>
          <specular>0 0 1 0.8</specular>
        </material>
      </visual>
      
      <collision name="chassis_collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.1</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Left wheel -->
    <link name="left_wheel">
      <pose>-0.15 -0.15 0 -1.5708 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
      
      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>
      
      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <!-- Right wheel -->
    <link name="right_wheel">
      <pose>-0.15 0.15 0 -1.5708 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.002</izz>
        </inertia>
      </inertial>
      
      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>
      
      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>
    
    <joint name="right_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>
    
    <!-- Differential drive plugin -->
    <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_radius>0.05</wheel_radius>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
    </plugin>
    
    <!-- Lidar sensor -->
    <link name="lidar_link">
      <pose>0.1 0 0.05 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      
      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
      
      <collision name="lidar_collision">
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.01</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <joint name="lidar_joint" type="fixed">
      <parent>chassis</parent>
      <child>lidar_link</child>
      <pose>0.1 0 0.05 0 0 0</pose>
    </joint>
    
    <sensor name="lidar" type="ray">
      <pose>0.1 0 0.05 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
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
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
    </sensor>
  </model>
</sdf>
```

### Model Configuration File

Create a configuration file for your model:

**File: `~/gazebo_ws/src/robot_models/models/differential_drive_robot/model.config`**

```xml
<?xml version="1.0"?>
<model>
  <name>Differential Drive Robot</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  
  <description>
    A simple differential drive robot with lidar sensor for simulation.
  </description>
</model>
```

## Advanced Robot Modeling

### Multi-body Systems

For complex robots like manipulators, you need to model multiple interconnected links:

**File: `~/gazebo_ws/src/robot_models/urdf/manipulator_robot.urdf.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="manipulator_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="0.2"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="0.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint 1: Base to shoulder -->
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI}" upper="${M_PI}" effort="100" velocity="1.0"/>
    <dynamics damping="1" friction="1"/>
  </joint>
  
  <!-- Shoulder link -->
  <link name="shoulder_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint 2: Shoulder to elbow -->
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.0"/>
    <dynamics damping="1" friction="1"/>
  </joint>
  
  <!-- Elbow link -->
  <link name="elbow_link">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint 3: Elbow to wrist -->
  <joint name="elbow_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50" velocity="1.0"/>
    <dynamics damping="0.5" friction="0.5"/>
  </joint>
  
  <!-- Wrist link -->
  <link name="wrist_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint 4: Wrist rotation -->
  <joint name="wrist_roll_joint" type="revolute">
    <parent link="wrist_link"/>
    <child link="gripper_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI}" upper="${M_PI}" effort="20" velocity="2.0"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>
  
  <!-- Gripper link -->
  <link name="gripper_link">
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>
  
  <gazebo reference="shoulder_link">
    <material>Gazebo/Red</material>
  </gazebo>
  
  <gazebo reference="elbow_link">
    <material>Gazebo/Green</material>
  </gazebo>
  
  <gazebo reference="wrist_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="gripper_link">
    <material>Gazebo/Yellow</material>
  </gazebo>
  
  <!-- Joint position controllers -->
  <gazebo>
    <plugin filename="libignition-gazebo-joint-position-controller-system.so" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>shoulder_pan_joint</joint_name>
      <topic>shoulder_pan_position</topic>
    </plugin>
    
    <plugin filename="libignition-gazebo-joint-position-controller-system.so" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>shoulder_lift_joint</joint_name>
      <topic>shoulder_lift_position</topic>
    </plugin>
    
    <plugin filename="libignition-gazebo-joint-position-controller-system.so" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>elbow_joint</joint_name>
      <topic>elbow_position</topic>
    </plugin>
    
    <plugin filename="libignition-gazebo-joint-position-controller-system.so" name="ignition::gazebo::systems::JointPositionController">
      <joint_name>wrist_roll_joint</joint_name>
      <topic>wrist_roll_position</topic>
    </plugin>
  </gazebo>
</robot>
```

## Environment Design Principles

### Physics-Based Environment Design

Creating realistic environments requires attention to physics properties:

**1. Collision Properties**: Define appropriate collision geometries for all objects
**2. Friction Coefficients**: Set realistic friction values for different surfaces
**3. Material Properties**: Use appropriate materials for realistic interactions
**4. Mass Distribution**: Ensure objects have realistic mass properties

### Creating Complex Environments

**File: `~/gazebo_ws/src/environments/worlds/complex_office.sdf`**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="complex_office">
    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
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
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Office furniture -->
    <!-- Desk -->
    <model name="office_desk">
      <pose>2 0 0 0 0 0</pose>
      <link name="desk_base">
        <inertial>
          <mass>50.0</mass>
          <inertia>
            <ixx>10.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>10.0</iyy>
            <iyz>0</iyz>
            <izz>15.0</izz>
          </inertia>
        </inertial>
        
        <visual name="desk_visual">
          <geometry>
            <box>
              <size>1.5 0.75 0.75</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
            <specular>0.8 0.6 0.2 1</specular>
          </material>
        </visual>
        
        <collision name="desk_collision">
          <geometry>
            <box>
              <size>1.5 0.75 0.75</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Chair -->
    <model name="office_chair">
      <pose>1.5 -0.5 0 0 0 0</pose>
      <link name="chair_base">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.5</izz>
          </inertia>
        </inertial>
        
        <visual name="chair_visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
            <specular>0.2 0.2 0.8 1</specular>
          </material>
        </visual>
        
        <collision name="chair_collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Bookshelf -->
    <model name="bookshelf">
      <pose>-2 1.5 0 0 0 0</pose>
      <link name="shelf_base">
        <inertial>
          <mass>30.0</mass>
          <inertia>
            <ixx>5.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5.0</iyy>
            <iyz>0</iyz>
            <izz>8.0</izz>
          </inertia>
        </inertial>
        
        <visual name="shelf_visual">
          <geometry>
            <box>
              <size>0.3 1.2 1.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.6 0.4 0.2 1</specular>
          </material>
        </visual>
        
        <collision name="shelf_collision">
          <geometry>
            <box>
              <size>0.3 1.2 1.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Dynamic objects -->
    <!-- Ball that can be manipulated -->
    <model name="red_ball">
      <pose>-1 0 0.1 0 0 0</pose>
      <link name="ball_link">
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        
        <visual name="ball_visual">
          <geometry>
            <sphere>
              <radius>0.05</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>1 0 0 1</specular>
          </material>
        </visual>
        
        <collision name="ball_collision">
          <geometry>
            <sphere>
              <radius>0.05</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Walls -->
    <model name="wall_1">
      <pose>0 3 0.5 0 0 0</pose>
      <link name="wall_link">
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>50.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>50.0</iyy>
            <iyz>0</iyz>
            <izz>100.0</izz>
          </inertia>
        </inertial>
        
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.7 0.7 0.7 1</specular>
          </material>
        </visual>
        
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Light source -->
    <light name="room_light" type="point">
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>10</range>
        <linear>0.1</linear>
        <quadratic>0.01</quadratic>
      </attenuation>
    </light>
  </world>
</sdf>
```

## Custom Gazebo Plugins

### Creating a Custom Plugin

Gazebo plugins extend simulation capabilities. Here's an example of a simple plugin that adds a custom behavior to a model:

**File: `~/gazebo_ws/src/gazebo_plugins/src/oscillating_motion_plugin.cpp`**

```cpp
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <ignition/transport/Node.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Angle.hh>

#include <iostream>

namespace gazebo_plugins
{
  class OscillatingMotionPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
  {
    public: void Configure(const ignition::gazebo::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          ignition::gazebo::EntityComponentManager &_ecm,
                          ignition::gazebo::EventManager &_eventMgr) override
    {
      // Get the model
      this->model = ignition::gazebo::Model(_entity);
      
      // Get parameters from SDF
      if (_sdf->HasElement("amplitude"))
      {
        this->amplitude = _sdf->Get<double>("amplitude");
      }
      
      if (_sdf->HasElement("frequency"))
      {
        this->frequency = _sdf->Get<double>("frequency");
      }
      
      if (_sdf->HasElement("axis"))
      {
        std::string axis_str = _sdf->Get<std::string>("axis");
        if (axis_str == "x")
          this->axis = ignition::math::Vector3d(1, 0, 0);
        else if (axis_str == "y")
          this->axis = ignition::math::Vector3d(0, 1, 0);
        else if (axis_str == "z")
          this->axis = ignition::math::Vector3d(0, 0, 1);
        else
          this->axis = ignition::math::Vector3d(1, 0, 0); // default to x-axis
      }
      
      // Store initial pose
      auto poseComp = _ecm.Component<ignition::gazebo::components::Pose>(_entity);
      if (poseComp)
      {
        this->initialPose = poseComp->Data();
      }
    }
    
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm) override
    {
      IGN_PROFILE("OscillatingMotionPlugin::PreUpdate");
      
      // Calculate oscillation based on time
      double time = std::chrono::duration<double>(_info.simTime).count();
      double oscillation = this->amplitude * sin(2 * M_PI * this->frequency * time);
      
      // Calculate new position
      ignition::math::Pose3d newPose = this->initialPose;
      newPose.Pos() = this->initialPose.Pos() + this->axis * oscillation;
      
      // Update the model's world pose
      _ecm.SetComponentData<ignition::gazebo::components::WorldPoseCmd>(
        this->model.Entity(), newPose);
    }
    
    private: ignition::gazebo::Model model;
    private: double amplitude = 0.1;  // meters
    private: double frequency = 1.0;  // Hz
    private: ignition::math::Vector3d axis = ignition::math::Vector3d::UnitX;
    private: ignition::math::Pose3d initialPose;
  };
}

// Register plugin
IGNITION_ADD_PLUGIN(
  gazebo_plugins::OscillatingMotionPlugin,
  ignition::gazebo::System,
  gazebo_plugins::OscillatingMotionPlugin::ISystemConfigure,
  gazebo_plugins::OscillatingMotionPlugin::ISystemPreUpdate)
```

### Using the Custom Plugin in a Model

**File: `~/gazebo_ws/src/robot_models/models/oscillating_object/model.sdf`**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="oscillating_object">
    <link name="object_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      
      <visual name="object_visual">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
      
      <collision name="object_collision">
        <geometry>
          <sphere>
            <radius>0.05</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    
    <plugin filename="liboscillating_motion_plugin.so" name="gazebo_plugins::OscillatingMotionPlugin">
      <amplitude>0.2</amplitude>
      <frequency>0.5</frequency>
      <axis>x</axis>
    </plugin>
  </model>
</sdf>
```

## Sensor Integration

### Camera Sensor Configuration

```xml
<sensor name="camera" type="camera">
  <pose>0.2 0 0.1 0 0 0</pose>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### IMU Sensor Configuration

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.1 0 0 0</pose>
  <topic>imu</topic>
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Model Validation Techniques

### Kinematic Validation

Verify that your model's kinematics are correct:

1. **Check joint limits**: Ensure joint limits are appropriate for the physical robot
2. **Verify transformations**: Check that all link transformations are correct
3. **Test range of motion**: Ensure the model can achieve all required poses

### Dynamic Validation

1. **Mass properties**: Verify that mass and inertia values are realistic
2. **Stability**: Check that the robot remains stable under gravity
3. **Contact behavior**: Verify that contacts behave as expected

### Sensor Validation

1. **Field of view**: Ensure sensors have appropriate field of view
2. **Range**: Verify that sensor ranges match real-world specifications
3. **Noise models**: Check that noise characteristics match real sensors

## Best Practices for Model Design

### 1. Accuracy vs. Performance

Balance model accuracy with simulation performance:
- Use simplified collision geometries where possible
- Reduce the number of complex meshes
- Optimize update rates for sensors

### 2. Modularity

Design models to be modular and reusable:
- Use xacro macros for common components
- Create parameterized models
- Organize models in a logical directory structure

### 3. Documentation

Document your models thoroughly:
- Include README files with model descriptions
- Document all parameters and their ranges
- Provide examples of how to use the model

### 4. Validation

Regularly validate your models:
- Compare simulation behavior with real-world data
- Test edge cases and failure modes
- Verify that models behave as expected under various conditions

## Troubleshooting Common Issues

### Issue 1: Robot Fails to Simulate Properly

**Symptoms**: Robot falls through the ground, behaves erratically
**Solutions**:
1. Check mass properties and inertial tensors
2. Verify collision geometries are properly defined
3. Ensure friction coefficients are set appropriately

### Issue 2: Sensors Not Working

**Symptoms**: Sensor data is not publishing or is incorrect
**Solutions**:
1. Verify sensor configuration in the model file
2. Check that the sensor plugin is correctly specified
3. Ensure the sensor is positioned appropriately

### Issue 3: Performance Issues

**Symptoms**: Simulation runs slowly or with low real-time factor
**Solutions**:
1. Simplify collision geometries
2. Reduce sensor update rates
3. Optimize physics parameters

## Summary

Creating realistic robot models and environments is essential for effective simulation in robotics. This chapter covered the fundamental techniques for designing both simple and complex robot models, creating detailed environments, and integrating sensors and custom behaviors.

Key takeaways include:
- Understanding the differences between URDF and SDF formats
- Creating accurate kinematic and dynamic models
- Designing realistic environments with appropriate physics properties
- Implementing custom plugins for specialized behaviors
- Validating models to ensure accuracy and performance

In the next chapter, we'll explore physics simulation and collision detection in detail, examining how to configure physics parameters for realistic behavior and how to handle complex collision scenarios.