# Chapter 2.4: Building Your First ROS 2 Package

## Learning Objectives

By the end of this chapter, students will be able to:
- Create a new ROS 2 package with proper structure and dependencies
- Implement nodes with publishers and subscribers
- Build and run ROS 2 packages using colcon
- Use launch files to manage complex system configurations
- Apply proper C++ and Python coding practices for ROS 2
- Debug common package creation and build issues

## Introduction

Creating your first ROS 2 package is a significant milestone in your journey to developing physical AI systems. A package is the fundamental unit of code organization in ROS 2, containing nodes, libraries, and other resources needed for your robotic applications. This chapter provides a step-by-step guide to creating, building, and running your first complete ROS 2 package.

Throughout this chapter, we'll create a practical example package that demonstrates key concepts in physical AI by implementing a simple robot controller that processes sensor data and generates motion commands. This will serve as a foundation for more complex systems we'll build throughout the course.

## Package Structure Overview

A ROS 2 package typically has the following structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml            # Package metadata and dependencies
├── src/                   # Source code files
│   ├── publisher_node.cpp
│   └── subscriber_node.cpp
├── include/               # Header files (C++)
├── launch/                # Launch files
├── config/                # Configuration files
├── scripts/               # Python scripts
├── test/                  # Test files
└── README.md              # Documentation
```

## Creating a New Package

### Step 1: Navigate to Your Workspace

```bash
cd ~/physical_ai_ws/src
```

### Step 2: Create the Package

We'll create a package called `physical_ai_basics` that will contain our first nodes:

```bash
ros2 pkg create --cpp --description "Basic examples for Physical AI concepts" physical_ai_basics
```

This command creates a basic package structure with necessary files.

### Step 3: Examine the Generated Structure

```bash
cd ~/physical_ai_ws/src/physical_ai_basics
ls -la
```

You'll see the following generated files:
- `CMakeLists.txt`: Build configuration file
- `package.xml`: Package metadata
- `src/physical_ai_basics.cpp`: A basic example node

## Package Metadata (package.xml)

Let's examine and update the `package.xml` file:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>physical_ai_basics</name>
  <version>0.0.0</version>
  <description>Basic examples for Physical AI concepts</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Key Elements in package.xml:

- **name**: The package name
- **version**: Package version (following semantic versioning)
- **description**: Brief description of the package
- **maintainer**: Contact information for the package maintainer
- **license**: Software license
- **depend**: Dependencies required by the package
- **buildtool_depend**: Build system dependencies

## Build Configuration (CMakeLists.txt)

The generated `CMakeLists.txt` file needs to be updated for our specific needs:

```cmake
cmake_minimum_required(VERSION 3.8)
project(physical_ai_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create the publisher node
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Create the subscriber node
add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Create the sensor processor node
add_executable(sensor_processor_node src/sensor_processor_node.cpp)
ament_target_dependencies(sensor_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  publisher_node
  subscriber_node
  sensor_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Creating Your First Node: Publisher

Let's create a publisher node that simulates sensor data:

**File: `src/publisher_node.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        // Create a publisher for laser scan messages
        laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        
        // Create a publisher for velocity commands
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create a timer to periodically publish data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Publish every 100ms
            std::bind(&PublisherNode::publish_data, this));
        
        RCLCPP_INFO(this->get_logger(), "Publisher node initialized");
    }

private:
    void publish_data()
    {
        // Publish laser scan data
        auto laser_msg = sensor_msgs::msg::LaserScan();
        laser_msg.header.stamp = this->get_clock()->now();
        laser_msg.header.frame_id = "laser_frame";
        laser_msg.angle_min = -1.57;  // -90 degrees
        laser_msg.angle_max = 1.57;   // 90 degrees
        laser_msg.angle_increment = 0.01745; // ~1 degree
        laser_msg.time_increment = 0.0;
        laser_msg.scan_time = 0.1;    // 100ms per scan
        laser_msg.range_min = 0.1;
        laser_msg.range_max = 10.0;
        
        // Generate sample laser data
        size_t num_readings = static_cast<size_t>((laser_msg.angle_max - laser_msg.angle_min) 
                                                / laser_msg.angle_increment);
        laser_msg.ranges.resize(num_readings);
        
        for (size_t i = 0; i < num_readings; ++i) {
            // Simulate an obstacle at a fixed distance
            laser_msg.ranges[i] = 2.0 + 0.5 * sin(i * 0.1);
        }
        
        laser_publisher_->publish(laser_msg);
        
        // Publish a simple velocity command
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.5;   // Move forward at 0.5 m/s
        cmd_msg.angular.z = 0.0;  // No rotation
        cmd_publisher_->publish(cmd_msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Creating Your First Node: Subscriber

Now let's create a subscriber node that receives and processes the published data:

**File: `src/subscriber_node.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        // Create a subscription for laser scan messages
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SubscriberNode::laser_callback, this, std::placeholders::_1));
        
        // Create a subscription for velocity commands
        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SubscriberNode::cmd_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Subscriber node initialized");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received laser scan with %zu ranges, min: %.2f, max: %.2f",
            msg->ranges.size(),
            msg->range_min,
            msg->range_max
        );
        
        // Find the minimum distance in the scan
        float min_distance = std::numeric_limits<float>::max();
        for (const auto& range : msg->ranges) {
            if (range < min_distance && range > msg->range_min) {
                min_distance = range;
            }
        }
        
        if (min_distance < 1.0) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f meters!", min_distance);
        } else {
            RCLCPP_INFO(this->get_logger(), "Clear path, obstacle at %.2f meters", min_distance);
        }
    }
    
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received velocity command: linear.x=%.2f, angular.z=%.2f",
            msg->linear.x,
            msg->angular.z
        );
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Creating a More Complex Node: Sensor Processor

Let's create a more sophisticated node that processes sensor data and generates appropriate responses:

**File: `src/sensor_processor_node.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SensorProcessorNode : public rclcpp::Node
{
public:
    SensorProcessorNode() : Node("sensor_processor_node")
    {
        // Create a subscription for laser scan messages
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SensorProcessorNode::scan_callback, this, std::placeholders::_1));
        
        // Create a publisher for velocity commands
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Sensor processor node initialized");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process the laser scan to find the closest obstacle
        float min_distance = std::numeric_limits<float>::max();
        int min_index = -1;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < min_distance && 
                msg->ranges[i] > msg->range_min && 
                msg->ranges[i] < msg->range_max) {
                
                min_distance = msg->ranges[i];
                min_index = i;
            }
        }
        
        // Create a velocity command based on the scan
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        if (min_index != -1 && min_distance < 1.0) {
            // Obstacle detected - turn away from obstacle
            float angle_to_obstacle = msg->angle_min + min_index * msg->angle_increment;
            
            if (angle_to_obstacle < 0) {
                // Obstacle on the left - turn right
                cmd_msg.linear.x = 0.2;   // Slow down
                cmd_msg.angular.z = -0.5; // Turn right
            } else {
                // Obstacle on the right - turn left
                cmd_msg.linear.x = 0.2;   // Slow down
                cmd_msg.angular.z = 0.5;  // Turn left
            }
            
            RCLCPP_WARN(
                this->get_logger(),
                "Obstacle at %.2f m, angle %.2f rad, turning accordingly",
                min_distance, angle_to_obstacle
            );
        } else {
            // Clear path - move forward
            cmd_msg.linear.x = 0.5;   // Move forward at 0.5 m/s
            cmd_msg.angular.z = 0.0;  // No rotation
            RCLCPP_INFO(this->get_logger(), "Clear path, moving forward");
        }
        
        cmd_publisher_->publish(cmd_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Building the Package

### Step 1: Navigate to Workspace Root

```bash
cd ~/physical_ai_ws
```

### Step 2: Build the Package

```bash
colcon build --packages-select physical_ai_basics
```

This command builds only the `physical_ai_basics` package, which is faster than building the entire workspace.

### Step 3: Source the Setup Files

```bash
source install/setup.bash
```

## Running the Package

### Option 1: Run Nodes Separately

Terminal 1:
```bash
ros2 run physical_ai_basics publisher_node
```

Terminal 2:
```bash
ros2 run physical_ai_basics subscriber_node
```

### Option 2: Run the Sensor Processor Node

```bash
ros2 run physical_ai_basics sensor_processor_node
```

## Creating Launch Files

Launch files allow you to start multiple nodes with a single command. Create a `launch` directory:

```bash
mkdir -p ~/physical_ai_ws/src/physical_ai_basics/launch
```

Create `sensor_demo.launch.py`:

**File: `launch/sensor_demo.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='physical_ai_basics',
            executable='publisher_node',
            name='sensor_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='physical_ai_basics',
            executable='sensor_processor_node',
            name='sensor_processor',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        ),
        Node(
            package='physical_ai_basics',
            executable='subscriber_node',
            name='sensor_subscriber',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])
```

### Running with Launch Files

```bash
cd ~/physical_ai_ws
source install/setup.bash
ros2 launch physical_ai_basics sensor_demo.launch.py
```

## Adding Parameters

Let's enhance our sensor processor node with parameters to make it configurable:

**Updated `src/sensor_processor_node.cpp` with parameters:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SensorProcessorNode : public rclcpp::Node
{
public:
    SensorProcessorNode() : Node("sensor_processor_node")
    {
        // Declare parameters with default values
        this->declare_parameter<float>("safety_distance", 1.0);
        this->declare_parameter<float>("linear_speed", 0.5);
        this->declare_parameter<float>("angular_speed", 0.5);
        this->declare_parameter<float>("slowdown_distance", 2.0);
        
        // Get parameter values
        safety_distance_ = this->get_parameter("safety_distance").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        slowdown_distance_ = this->get_parameter("slowdown_distance").as_double();
        
        // Create a subscription for laser scan messages
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SensorProcessorNode::scan_callback, this, std::placeholders::_1));
        
        // Create a publisher for velocity commands
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "Sensor processor node initialized with safety_distance=%.2f", 
            safety_distance_);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process the laser scan to find the closest obstacle
        float min_distance = std::numeric_limits<float>::max();
        int min_index = -1;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < min_distance && 
                msg->ranges[i] > msg->range_min && 
                msg->ranges[i] < msg->range_max) {
                
                min_distance = msg->ranges[i];
                min_index = i;
            }
        }
        
        // Create a velocity command based on the scan
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        if (min_index != -1 && min_distance < safety_distance_) {
            // Obstacle detected - turn away from obstacle
            float angle_to_obstacle = msg->angle_min + min_index * msg->angle_increment;
            
            if (angle_to_obstacle < 0) {
                // Obstacle on the left - turn right
                cmd_msg.linear.x = linear_speed_ * 0.4;   // Slow down significantly
                cmd_msg.angular.z = -angular_speed_;      // Turn right
            } else {
                // Obstacle on the right - turn left
                cmd_msg.linear.x = linear_speed_ * 0.4;   // Slow down significantly
                cmd_msg.angular.z = angular_speed_;       // Turn left
            }
            
            RCLCPP_WARN(
                this->get_logger(),
                "Obstacle at %.2f m, angle %.2f rad, turning accordingly",
                min_distance, angle_to_obstacle
            );
        } else if (min_index != -1 && min_distance < slowdown_distance_) {
            // Obstacle detected but at a safe distance - slow down
            cmd_msg.linear.x = linear_speed_ * 0.7;   // Moderate speed
            cmd_msg.angular.z = 0.0;                  // No rotation
            RCLCPP_INFO(this->get_logger(), 
                "Obstacle at safe distance %.2f m, reducing speed", min_distance);
        } else {
            // Clear path - move forward at normal speed
            cmd_msg.linear.x = linear_speed_;  // Normal speed
            cmd_msg.angular.z = 0.0;           // No rotation
            RCLCPP_INFO(this->get_logger(), "Clear path, moving forward at normal speed");
        }
        
        cmd_publisher_->publish(cmd_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    
    // Parameters
    float safety_distance_;
    float linear_speed_;
    float angular_speed_;
    float slowdown_distance_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Creating a Parameter Configuration File

Create `config/sensor_params.yaml`:

```yaml
/**:
  ros__parameters:
    safety_distance: 1.0
    linear_speed: 0.5
    angular_speed: 0.8
    slowdown_distance: 2.0
```

## Building and Testing with Parameters

### Step 1: Update CMakeLists.txt to Install Config Files

Add to `CMakeLists.txt` before `ament_package()`:

```cmake
# Install configuration files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

### Step 2: Rebuild the Package

```bash
cd ~/physical_ai_ws
colcon build --packages-select physical_ai_basics
source install/setup.bash
```

### Step 3: Run with Parameters

```bash
# Method 1: Use the parameter file
ros2 run physical_ai_basics sensor_processor_node --ros-args --params-file ./install/physical_ai_basics/share/physical_ai_basics/config/sensor_params.yaml

# Method 2: Set parameters at runtime
ros2 run physical_ai_basics sensor_processor_node --ros-args -p safety_distance:=1.5 -p linear_speed:=0.7
```

## Python Nodes

Let's also create a Python version of one of our nodes to demonstrate Python in ROS 2:

Create `scripts/python_sensor_subscriber.py`:

```python
#!/usr/bin/env python3
# python_sensor_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PythonSensorSubscriber(Node):
    def __init__(self):
        super().__init__('python_sensor_subscriber')
        
        # Create subscription for laser scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.safety_distance = 1.0
        self.linear_speed = 0.5
        
        self.get_logger().info('Python sensor subscriber initialized')

    def scan_callback(self, msg):
        # Find minimum distance in scan
        min_distance = float('inf')
        for range_val in msg.ranges:
            if msg.range_min < range_val < msg.range_max and range_val < min_distance:
                min_distance = range_val
        
        # Create velocity command based on scan
        cmd_msg = Twist()
        
        if min_distance < self.safety_distance:
            # Obstacle detected - stop
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f} meters! Stopping.')
        else:
            # Clear path - move forward
            cmd_msg.linear.x = self.linear_speed
            cmd_msg.angular.z = 0.0
            self.get_logger().info(f'Clear path, obstacle at {min_distance:.2f} meters')
        
        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PythonSensorSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Update CMakeLists.txt for Python Scripts

Add to `CMakeLists.txt` before `ament_package()`:

```cmake
# Install Python scripts
cat_install_python(
  PROGRAMS
    scripts/python_sensor_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)
```

## Testing Your Package

### Step 1: Verify Package Installation

```bash
# List available packages
ros2 pkg list | grep physical_ai_basics

# Get information about your package
ros2 pkg info physical_ai_basics
```

### Step 2: List Available Executables

```bash
# List executables in your package
ros2 run physical_ai_basics
```

### Step 3: Test Communication

```bash
# In one terminal, run the publisher
ros2 run physical_ai_basics publisher_node

# In another terminal, check what topics are available
ros2 topic list

# In another terminal, echo the scan topic
ros2 topic echo /scan

# In another terminal, echo the cmd_vel topic
ros2 topic echo /cmd_vel
```

## Common Issues and Troubleshooting

### 1. Build Issues

**Problem:** "Package not found" after building
**Solution:** Make sure to source the setup files:
```bash
source install/setup.bash
```

**Problem:** "CMake Error" during build
**Solution:** Check your CMakeLists.txt for syntax errors and ensure all dependencies are declared in package.xml

### 2. Runtime Issues

**Problem:** "Node not found" when trying to run
**Solution:** Ensure the package was built successfully and the setup files are sourced

**Problem:** Nodes can't communicate
**Solution:** Check that nodes are on the same ROS domain and that topic names match exactly

### 3. Permissions Issues

**Problem:** Python scripts not executable
**Solution:** Make sure scripts have execute permissions:
```bash
chmod +x scripts/python_sensor_subscriber.py
```

## Best Practices for Package Development

### 1. Code Organization
- Keep related functionality in the same package
- Use descriptive names for packages, nodes, and topics
- Separate headers and implementations properly

### 2. Documentation
- Update the package description in package.xml
- Include a README.md with usage instructions
- Comment your code appropriately

### 3. Testing
- Test nodes individually before integration
- Use launch files for complex system testing
- Implement parameter validation

### 4. Version Control
- Use Git to track changes to your package
- Include a .gitignore file to exclude build artifacts
- Commit changes regularly with descriptive messages

## Summary

In this chapter, we've created our first complete ROS 2 package that demonstrates fundamental concepts in physical AI. We've covered:

- The structure and components of a ROS 2 package
- Creating nodes with publishers and subscribers
- Building packages with colcon
- Using launch files for system management
- Implementing parameters for configurability
- Creating both C++ and Python nodes

The package we created simulates a simple robot controller that processes sensor data and generates motion commands, embodying the principles of embodied intelligence through the sensorimotor loop. This serves as a foundation for more complex physical AI systems we'll develop throughout the course.

In the next chapter, we'll explore advanced ROS 2 concepts including services, actions, and more complex message types.