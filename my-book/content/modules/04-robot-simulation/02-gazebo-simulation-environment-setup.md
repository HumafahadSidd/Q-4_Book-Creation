# Chapter 4.2: Gazebo Simulation Environment Setup

## Learning Objectives

By the end of this chapter, students will be able to:
- Install and configure Gazebo simulation environment
- Understand the Gazebo architecture and components
- Set up a basic simulation environment with a robot model
- Configure physics parameters and sensor models
- Launch and run simulation scenarios
- Troubleshoot common Gazebo setup issues

## Introduction

Gazebo is a powerful, open-source robotics simulator that provides high-fidelity physics simulation, realistic rendering, and convenient programmatic interfaces. It is widely used in robotics research and development for testing algorithms, validating system designs, and training perception systems before deployment on real hardware.

In this chapter, we'll walk through the complete setup process for Gazebo, from installation to creating your first simulation environment. We'll cover the essential components of Gazebo, how to configure it for different types of robotic applications, and best practices for setting up simulation environments.

Gazebo integrates well with ROS and ROS 2, making it an ideal choice for physical AI development within the ROS ecosystem. The combination of Gazebo and ROS provides a complete framework for developing, testing, and validating robotic systems.

## Gazebo Architecture

### Core Components

Gazebo consists of several core components that work together to provide a complete simulation environment:

**Gazebo Server (gzserver)**: The core simulation engine that handles physics simulation, sensor models, and plugin execution. It runs in the background and manages the simulation state.

**Gazebo Client (gzclient)**: The graphical user interface that provides visualization of the simulation. It connects to the server to display the 3D environment and robot states.

**Physics Engine**: Gazebo supports multiple physics engines including ODE (Open Dynamics Engine), Bullet, and Simbody. These engines handle collision detection, contact processing, and dynamic simulation.

**Sensor Models**: Built-in models for various sensors including cameras, LIDAR, IMUs, force/torque sensors, and more. These models simulate the behavior of real sensors with appropriate noise characteristics.

**Plugin System**: A flexible plugin architecture that allows users to extend Gazebo's functionality with custom models, controllers, and sensors.

### Gazebo vs Ignition Robotics

Gazebo is transitioning to Ignition Robotics, which provides a more modular and modern architecture. As of the latest developments:

- **Gazebo Classic**: The traditional monolithic Gazebo application
- **Ignition Robotics**: A collection of modular libraries and tools
- **Gazebo Garden/Harmonic**: Newer versions that are part of the Ignition ecosystem

For this course, we'll focus on the latest stable version of Gazebo, which is part of the Ignition ecosystem.

## Installing Gazebo

### System Requirements

Before installing Gazebo, ensure your system meets the following requirements:

**Minimum Requirements:**
- Operating System: Ubuntu 22.04 LTS or later, Windows 11 with WSL2, or macOS with Docker
- Processor: Multi-core processor (Intel i5 or equivalent)
- Memory: 8 GB RAM (16 GB recommended)
- Graphics: GPU with OpenGL 3.3 support
- Storage: 5 GB free disk space

**Recommended Requirements:**
- Operating System: Ubuntu 22.04 LTS
- Processor: Intel i7 or AMD Ryzen 7 (or better)
- Memory: 16 GB RAM (32 GB for complex simulations)
- Graphics: Dedicated GPU with CUDA support
- Storage: SSD with 20 GB free space

### Installation on Ubuntu 22.04

1. **Set up the repository**
   ```bash
   sudo apt update
   sudo apt install wget lsb-release gnupg
   sudo sh -c 'echo "deb [arch=amd64] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt update
   ```

2. **Install Gazebo Garden**
   ```bash
   sudo apt install gz-garden
   ```

3. **Install ROS 2 Gazebo packages**
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

4. **Verify installation**
   ```bash
   gz --version
   ```

### Installation on Windows with WSL2

1. **Install WSL2 with Ubuntu 22.04**
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

2. **Follow Ubuntu installation instructions** inside the WSL terminal

3. **Install X11 server for GUI support**
   - Install VcXsrv or WSLg
   - Configure for X11 forwarding

4. **Set up display forwarding in WSL**
   ```bash
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0
   ```

### Installation on macOS

1. **Install Homebrew** (if not already installed)
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. **Install Gazebo using Homebrew**
   ```bash
   brew install osrf/simulation/gz-garden
   ```

3. **Install additional dependencies**
   ```bash
   brew install cmake pkg-config eigen sdformat
   ```

## Configuring Gazebo

### Environment Variables

Gazebo uses several environment variables that can be configured for your setup:

```bash
# Add to your ~/.bashrc or ~/.zshrc
export GZ_SIM_RESOURCE_PATH=/usr/share/gazebo/worlds:/path/to/custom/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/gazebo/plugins
export GZ_SIM_MEDIA_PATH=/usr/share/gazebo/media
```

### Configuration File

Gazebo can be configured using a configuration file located at `~/.gz/sim/config.yaml`:

```yaml
version: 2
simulator_gui:
  plugins:
    - filename: "GzScene3D"
      name: "3D View"
      params:
        camera_pose: "0 -4 2 0 0.5 0"
    - filename: "WorldControl"
      name: "World Control"
    - filename: "WorldStats"
      name: "World Statistics"
```

## Creating Your First Simulation Environment

### Step 1: Create a Workspace

First, let's create a workspace for our simulation projects:

```bash
mkdir -p ~/gazebo_ws/src
cd ~/gazebo_ws
```

### Step 2: Create a Robot Model

Create a simple differential drive robot model using URDF (Unified Robot Description Format):

**File: `~/gazebo_ws/src/my_robot_description/urdf/my_robot.urdf`**

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Base to Left Wheel Joint -->
  <joint name="base_to_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.15 -0.2 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Base to Right Wheel Joint -->
  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.15 0.2 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Chassis to Lidar Joint -->
  <joint name="base_to_lidar" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.1"/>
  </joint>

  <!-- Lidar Link -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

### Step 3: Create Gazebo-Specific Model Files

Create a Gazebo model configuration file:

**File: `~/gazebo_ws/src/my_robot_description/model.sdf`**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="my_robot">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.2</iyy>
          <iyz>0</iyz>
          <izz>0.3</izz>
        </inertia>
      </inertial>
      <visual name="chassis_visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.15</size>
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
            <size>0.5 0.3 0.15</size>
          </box>
        </geometry>
      </collision>
      <sensor name="lidar" type="ray">
        <pose>0.15 0 0.1 0 0 0</pose>
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
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>
    
    <link name="left_wheel">
      <pose>-0.15 -0.2 0 0 1.5708 0</pose>
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
            <radius>0.1</radius>
            <length>0.05</length>
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
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <link name="right_wheel">
      <pose>-0.15 0.2 0 0 1.5708 0</pose>
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
            <radius>0.1</radius>
            <length>0.05</length>
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
            <radius>0.1</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
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
    
    <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_radius>0.1</wheel_radius>
      <odom_publish_frequency>30</odom_publish_frequency>
      <topic>cmd_vel</topic>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
    </plugin>
  </model>
</sdf>
```

### Step 4: Create a World File

Create a simple world file for our simulation:

**File: `~/gazebo_ws/src/my_robot_description/worlds/simple_world.sdf`**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add some obstacles -->
    <model name="box1">
      <pose>-1 1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>1 0 0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <model name="box2">
      <pose>1 -1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
            <specular>0 1 0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Place our robot in the world -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Step 5: Set Up the Model Path

To make Gazebo recognize your custom model, you need to set the model path:

```bash
# Add to your ~/.bashrc
export GZ_SIM_RESOURCE_PATH=$HOME/gazebo_ws/src/my_robot_description:$GZ_SIM_RESOURCE_PATH
```

## Launching Your First Simulation

### Method 1: Direct Gazebo Launch

```bash
# Launch the simulation with your custom world
gz sim -r simple_world.sdf
```

### Method 2: Using ROS 2 with Gazebo

Create a ROS 2 launch file to launch Gazebo with your robot:

**File: `~/gazebo_ws/src/my_robot_bringup/launch/robot_sim.launch.py`**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'worlds',
            'simple_world.sdf'
        ]),
        description='SDF world file'
    )
    
    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')]
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Bridge for communication between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        gz_sim,
        spawn_robot,
        bridge
    ])
```

### Method 3: Using Gazebo with Robot State Publisher

Create a more complete launch file that includes robot state publishing:

**File: `~/gazebo_ws/src/my_robot_bringup/launch/complete_sim.launch.py`**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'worlds',
            'simple_world.sdf'
        ]),
        description='SDF world file'
    )
    
    # Launch Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world')]
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'robot_description': '''
                <robot name="my_robot">
                    <link name="base_link">
                        <visual>
                            <geometry>
                                <box size="0.5 0.3 0.15"/>
                            </geometry>
                            <material name="blue">
                                <color rgba="0 0 1 0.8"/>
                            </material>
                        </visual>
                        <collision>
                            <geometry>
                                <box size="0.5 0.3 0.15"/>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass value="5.0"/>
                            <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
                        </inertial>
                    </link>
                    <link name="left_wheel">
                        <visual>
                            <geometry>
                                <cylinder radius="0.1" length="0.05"/>
                            </geometry>
                            <material name="black">
                                <color rgba="0 0 0 1"/>
                            </material>
                        </visual>
                        <collision>
                            <geometry>
                                <cylinder radius="0.1" length="0.05"/>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass value="0.5"/>
                            <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
                        </inertial>
                    </link>
                    <link name="right_wheel">
                        <visual>
                            <geometry>
                                <cylinder radius="0.1" length="0.05"/>
                            </geometry>
                            <material name="black">
                                <color rgba="0 0 0 1"/>
                            </material>
                        </visual>
                        <collision>
                            <geometry>
                                <cylinder radius="0.1" length="0.05"/>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass value="0.5"/>
                            <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
                        </inertial>
                    </link>
                    <link name="lidar_link">
                        <visual>
                            <geometry>
                                <cylinder radius="0.03" length="0.05"/>
                            </geometry>
                            <material name="red">
                                <color rgba="1 0 0 1"/>
                            </material>
                        </visual>
                        <collision>
                            <geometry>
                                <cylinder radius="0.03" length="0.05"/>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass value="0.1"/>
                            <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
                        </inertial>
                    </link>
                    <joint name="base_to_left_wheel" type="continuous">
                        <parent link="base_link"/>
                        <child link="left_wheel"/>
                        <origin xyz="-0.15 -0.2 0" rpy="1.5708 0 0"/>
                        <axis xyz="0 0 1"/>
                    </joint>
                    <joint name="base_to_right_wheel" type="continuous">
                        <parent link="base_link"/>
                        <child link="right_wheel"/>
                        <origin xyz="-0.15 0.2 0" rpy="1.5708 0 0"/>
                        <axis xyz="0 0 1"/>
                    </joint>
                    <joint name="base_to_lidar" type="fixed">
                        <parent link="base_link"/>
                        <child link="lidar_link"/>
                        <origin xyz="0.15 0 0.1"/>
                    </joint>
                </robot>
            '''
        }]
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Bridge for communication between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge
    ])
```

## Configuring Physics Parameters

### Physics Engine Configuration

Gazebo allows you to configure various physics parameters in your world file:

```xml
<world name="simple_world">
  <!-- Physics engine configuration -->
  <physics name="1ms" type="ode">
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
  
  <!-- Rest of the world configuration -->
</world>
```

### Material Properties

You can define custom material properties for more realistic simulation:

```xml
<world name="simple_world">
  <!-- Material definitions -->
  <material name="custom_blue">
    <ambient>0 0 0.8 1</ambient>
    <diffuse>0 0 1 1</diffuse>
    <specular>0.5 0.5 1 1</specular>
    <emissive>0 0 0 0</emissive>
  </material>
  
  <!-- Models using custom materials -->
  <model name="custom_box">
    <link name="link">
      <visual name="visual">
        <material>custom_blue</material>
        <!-- geometry definition -->
      </visual>
    </link>
  </model>
</world>
```

## Sensor Configuration

### Camera Sensor

Add a camera sensor to your robot model:

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
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### IMU Sensor

Add an IMU sensor to your robot:

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.1 0 0 0</pose>
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

## Troubleshooting Common Issues

### Issue 1: Gazebo Not Launching

**Symptom**: Gazebo fails to start with graphics errors
**Solution**: 
1. Check if you have proper graphics drivers installed
2. If using WSL2, ensure X11 forwarding is configured properly
3. Try running with software rendering: `gz sim --render-engine=ogre2`

### Issue 2: Robot Not Appearing in Simulation

**Symptom**: Robot model doesn't appear in the simulation
**Solution**:
1. Check that the model path is correctly set
2. Verify the model file syntax
3. Check that the spawn command is executed properly

### Issue 3: Sensor Data Not Publishing

**Symptom**: Sensor topics are not publishing data
**Solution**:
1. Check that the sensor is properly defined in the model
2. Verify the bridge configuration
3. Check that the update rate is set correctly

### Issue 4: Performance Issues

**Symptom**: Simulation running slowly
**Solution**:
1. Reduce the physics update rate
2. Simplify collision geometries
3. Reduce the number of objects in the scene
4. Use simpler physics engine parameters

## Testing Your Setup

### Basic Functionality Test

1. **Launch the simulation**:
   ```bash
   gz sim -r worlds/simple_world.sdf
   ```

2. **Check if your robot appears** in the simulation environment

3. **Verify physics simulation** by observing the robot's interaction with the environment

4. **Test sensor outputs** using Gazebo's built-in tools or ROS 2 tools

### ROS 2 Integration Test

1. **Launch the complete simulation**:
   ```bash
   cd ~/gazebo_ws
   source install/setup.bash
   ros2 launch my_robot_bringup complete_sim.launch.py
   ```

2. **Verify ROS 2 topics**:
   ```bash
   ros2 topic list | grep -E "(cmd_vel|odom|scan)"
   ```

3. **Send commands to the robot**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
   ```

4. **Monitor sensor data**:
   ```bash
   ros2 topic echo /scan
   ```

## Best Practices for Gazebo Setup

### 1. Model Organization

- Organize your models in a clear directory structure
- Use descriptive names for models and components
- Include README files with model descriptions

### 2. Physics Parameter Tuning

- Start with conservative physics parameters
- Gradually adjust for performance and accuracy
- Document parameter choices for reproducibility

### 3. Sensor Configuration

- Configure sensors with realistic noise models
- Set appropriate update rates for your application
- Validate sensor data against real-world characteristics

### 4. Performance Optimization

- Use simplified collision geometries when possible
- Adjust update rates based on requirements
- Monitor simulation real-time factor

### 5. Validation

- Regularly compare simulation results with real hardware
- Validate sensor models with actual sensor data
- Test edge cases in both simulation and reality

## Summary

Setting up Gazebo simulation environment is a critical step in developing physical AI systems. This chapter covered the complete process from installation to creating your first simulation environment with a robot model.

Key takeaways include:
- Understanding Gazebo's architecture and components
- Installing and configuring Gazebo for your development environment
- Creating robot models and world files
- Setting up ROS 2 integration with Gazebo
- Configuring physics parameters and sensors
- Troubleshooting common setup issues

In the next chapter, we'll explore creating robot models and environments in more detail, including advanced modeling techniques and environment design principles.