# Chapter 2.3: Message Types and Communication Patterns

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the structure and definition of ROS 2 message types
- Create custom message types for specific applications
- Select appropriate message types for different data streams
- Implement efficient communication patterns for physical AI systems
- Optimize message structures for performance and bandwidth
- Apply best practices for message design in robotic applications

## Introduction

In the previous chapter, we explored the fundamental communication mechanisms in ROS 2: nodes, topics, services, and actions. Now we'll dive deeper into the data structures that flow through these communication channels: message types. Understanding how to design and use appropriate message types is crucial for developing efficient and maintainable physical AI systems.

Messages are the lifeblood of ROS 2 systems, carrying sensor data, control commands, state information, and other critical data between nodes. The design of these messages directly impacts system performance, maintainability, and interoperability. In physical AI systems, where real-time performance and reliability are critical, careful message design becomes even more important.

## Standard Message Types Overview

ROS 2 provides a rich set of standard message types organized into packages for different purposes:

### std_msgs
Contains basic data types that are commonly used across all applications:
- `Bool`, `Int8`, `Int16`, `Int32`, `Int64`
- `UInt8`, `UInt16`, `UInt32`, `UInt64`
- `Float32`, `Float64`
- `String`
- `ColorRGBA`
- `Header` (contains timestamp and frame information)

### geometry_msgs
Contains messages for representing geometric data:
- `Point`: 3D point (x, y, z)
- `Pose`: Position and orientation (position + orientation)
- `Twist`: Linear and angular velocity (linear + angular)
- `Transform`: Position and orientation between coordinate frames
- `Quaternion`: Rotation representation (x, y, z, w)

### sensor_msgs
Contains messages for sensor data:
- `LaserScan`: 2D laser range finder data
- `Image`: Image data from cameras
- `PointCloud2`: 3D point cloud data
- `JointState`: Joint positions, velocities, and efforts
- `Imu`: Inertial measurement unit data
- `BatteryState`: Battery information

### nav_msgs
Contains messages for navigation:
- `Odometry`: Position and velocity with covariance
- `Path`: Sequence of poses
- `OccupancyGrid`: 2D occupancy grid map
- `MapMetaData`: Information about map structure

### trajectory_msgs
Contains messages for trajectory specifications:
- `JointTrajectory`: Desired joint positions over time
- `MultiDOFJointTrajectory`: Multi-degree-of-freedom trajectory

## Message Structure and Definition

### Message Definition Syntax

Messages are defined using the `.msg` file format with a simple syntax:

```
# Comments start with #
# Primitive field types
bool field_name1
int32 field_name2
float64 field_name3
string field_name4

# Array of primitive types
int32[] array_field

# Fixed-size array
int32[5] fixed_array_field

# Nested message types
geometry_msgs/Point position
geometry_msgs/Pose pose
```

### Example: Custom Sensor Message

Let's define a custom message for a multi-sensor unit:

```
# CustomSensorData.msg
# A message representing data from a multi-sensor unit

# Header for timestamp and frame information
std_msgs/Header header

# Sensor identification
string sensor_id

# Temperature reading
float64 temperature

# Multiple distance readings
float64[] distances

# Confidence in readings
float64 confidence

# Calibration status
bool calibrated
```

### Example: Robot State Message

```
# RobotState.msg
# A message representing the complete state of a robot

# Timestamp and frame
std_msgs/Header header

# Position and orientation
geometry_msgs/Pose pose

# Velocity information
geometry_msgs/Twist velocity

# Joint states
sensor_msgs/JointState joint_states

# Battery information
sensor_msgs/BatteryState battery

# Current task status
string current_task

# System status flags
bool emergency_stop
bool autonomous_mode
```

## Creating Custom Message Types

### Step 1: Define the Message

Create a `msg` directory in your package and define your message:

```bash
mkdir -p ~/physical_ai_ws/src/my_robot_msgs/msg
```

Create `CustomSensorData.msg` in the `msg` directory:

```
# Custom sensor data message
std_msgs/Header header
string sensor_id
float64 temperature
float64[] distances
float64 confidence
bool calibrated
```

### Step 2: Update CMakeLists.txt

Add the message generation dependencies to your `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

# Define your messages
set(msg_files
  "msg/CustomSensorData.msg"
  "msg/RobotState.msg"
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  ADD_LINTER_TESTS
)
```

### Step 3: Update package.xml

Add the necessary dependencies to your `package.xml`:

```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>

<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 4: Build the Package

```bash
cd ~/physical_ai_ws
colcon build --packages-select my_robot_msgs
source install/setup.bash
```

### Using Custom Messages in Code

Now you can use your custom message in C++:

```cpp
#include "my_robot_msgs/msg/custom_sensor_data.hpp"

void publish_custom_message() {
    auto msg = my_robot_msgs::msg::CustomSensorData();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "sensor_frame";
    msg.sensor_id = "temp_dist_01";
    msg.temperature = 25.5;
    msg.distances = {1.2, 1.5, 1.8, 2.1};
    msg.confidence = 0.95;
    msg.calibrated = true;
    
    publisher_->publish(msg);
}
```

## Message Design Principles

### 1. Efficiency and Performance

**Minimize Message Size:**
- Use appropriate data types (int8 vs int64 when possible)
- Consider fixed-size arrays vs variable-length arrays
- Remove unnecessary fields
- Compress large data when appropriate

**Example of efficient vs inefficient design:**

Inefficient:
```
# Inefficient message with unnecessary fields
string sensor_name
string sensor_type
string sensor_manufacturer
string sensor_model
string sensor_serial_number
int32 sensor_id
float64 temperature
float64 humidity
float64 pressure
float64 altitude
float64 latitude
float64 longitude
# ... many more fields
```

Efficient:
```
# Efficient message with essential fields only
std_msgs/Header header
int32 id
float64 temperature
float64[] values  # Combine related values
```

### 2. Extensibility

Design messages to accommodate future changes:

```
# Version 1
float64 temperature

# Version 2 (backward compatible)
float64 temperature
float64 humidity  # Added new field
float64 pressure  # Added new field
```

### 3. Semantic Clarity

Use descriptive field names that clearly indicate their purpose:

Good:
```
float64 linear_velocity_x
float64 angular_velocity_z
string status_description
```

Avoid:
```
float64 v1
float64 v2
string s
```

## Communication Patterns for Physical AI

### 1. Sensor Data Streaming Pattern

For continuous sensor data, use high-frequency publishing with appropriate QoS:

```cpp
// Publisher setup for sensor data
auto qos = rclcpp::QoS(10);  // history depth of 10
qos.best_effort();          // optimize for speed over reliability for sensor data
qos.durability_volatile();  // don't store for late joiners

auto sensor_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", qos);
```

### 2. State Synchronization Pattern

For sharing state information, use reliable communication:

```cpp
// Publisher setup for state information
auto qos = rclcpp::QoS(1);      // minimal history
qos.reliable();                 // ensure delivery
qos.transient_local();          // store for late joiners

auto state_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);
```

### 3. Command Pattern

For sending control commands, use reliable communication with appropriate timeouts:

```cpp
// Publisher setup for control commands
auto qos = rclcpp::QoS(1);
qos.reliable();
qos.durability_volatile();

auto cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
```

## Message Optimization Techniques

### 1. Data Compression

For large data like images or point clouds, consider compression:

```cpp
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>

void publish_compressed_image(const cv::Mat& image) {
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = this->get_clock()->now();
    cv_image.header.frame_id = "camera_frame";
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = image;
    
    sensor_msgs::msg::CompressedImage compressed_msg;
    cv_image.toCompressedImageMsg(compressed_msg, "jpeg");
    
    compressed_image_publisher_->publish(compressed_msg);
}
```

### 2. Data Sampling and Filtering

For high-frequency data, consider sampling or filtering to reduce bandwidth:

```cpp
// Example of temporal sampling
class SampledSensorNode : public rclcpp::Node
{
private:
    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Only process every Nth message
        if (++counter_ % sampling_rate_ == 0) {
            // Process and publish the message
            publisher_->publish(*msg);
        }
    }
    
    int counter_ = 0;
    int sampling_rate_ = 5;  // Publish every 5th message
};
```

### 3. Spatial Filtering

For sensor data, apply spatial filtering to reduce message size:

```cpp
// Example of spatial filtering for LIDAR data
void spatial_filter(const sensor_msgs::msg::LaserScan& input, 
                   sensor_msgs::msg::LaserScan& output, 
                   int skip_factor) {
    output = input;
    output.ranges.clear();
    
    for (size_t i = 0; i < input.ranges.size(); i += skip_factor) {
        output.ranges.push_back(input.ranges[i]);
    }
}
```

## Quality of Service (QoS) Optimization

### Reliability vs. Performance Trade-offs

**For Sensor Data:**
- Use `BEST_EFFORT` for real-time sensor streams where latest data is more important than all data
- Use smaller history queues to reduce memory usage

**For Control Commands:**
- Use `RELIABLE` to ensure commands are delivered
- Use `TRANSIENT_LOCAL` durability for important commands that late joiners should receive

**For State Information:**
- Use `RELIABLE` with `TRANSIENT_LOCAL` to ensure state information is available to new nodes

### Example QoS Configurations

```cpp
// High-frequency sensor data (camera, LIDAR)
auto sensor_qos = rclcpp::QoS(5);  // small history
sensor_qos.best_effort();
sensor_qos.durability_volatile();

// State information (odometry, joint states)
auto state_qos = rclcpp::QoS(1);
state_qos.reliable();
state_qos.transient_local();

// Control commands
auto cmd_qos = rclcpp::QoS(1);
cmd_qos.reliable();
cmd_qos.durability_volatile();
```

## Advanced Message Techniques

### 1. Message Aggregation

Combine multiple related messages into a single message to reduce overhead:

```
# RobotSensors.msg
std_msgs/Header header
sensor_msgs/LaserScan laser_scan
sensor_msgs/Imu imu_data
sensor_msgs/JointState joint_states
sensor_msgs/BatteryState battery
```

### 2. Message Splitting

For large messages, consider splitting into smaller, more focused messages:

Instead of:
```
# LargeState.msg
geometry_msgs/Pose pose
geometry_msgs/Twist twist
sensor_msgs/JointState joint_states
sensor_msgs/Imu imu
nav_msgs/OccupancyGrid map
# ... many more fields
```

Use separate topics:
- `/robot/pose` - geometry_msgs/Pose
- `/robot/twist` - geometry_msgs/Twist
- `/robot/joint_states` - sensor_msgs/JointState
- etc.

### 3. Message Validation

Implement validation for critical messages:

```cpp
bool validate_pose_message(const geometry_msgs::msg::Pose& pose) {
    // Check for NaN values
    if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z)) {
        return false;
    }
    
    // Check quaternion normalization
    double norm = pose.orientation.x * pose.orientation.x +
                  pose.orientation.y * pose.orientation.y +
                  pose.orientation.z * pose.orientation.z +
                  pose.orientation.w * pose.orientation.w;
                  
    if (std::abs(norm - 1.0) > 0.01) {
        return false;
    }
    
    return true;
}
```

## Best Practices for Message Design

### 1. Follow Established Conventions

- Use standard message types when possible
- Follow naming conventions (lowercase with underscores)
- Include appropriate headers with timestamps and frame IDs
- Use consistent units (SI units preferred)

### 2. Plan for Evolution

- Design messages that can be extended without breaking compatibility
- Consider using version fields for critical messages
- Document message semantics clearly

### 3. Consider Network Implications

- Estimate bandwidth requirements for high-frequency topics
- Use appropriate QoS settings for your network topology
- Consider message size when designing for wireless networks

### 4. Maintain Type Safety

- Use strongly typed messages rather than generic containers
- Validate message contents in critical applications
- Use ROS 2's type checking features during development

## Performance Monitoring

### Measuring Message Performance

```cpp
// Example of measuring message latency
class LatencyMonitor : public rclcpp::Node
{
public:
    LatencyMonitor() : Node("latency_monitor")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/timestamp_test", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/timestamp_test_echo",
            10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                auto now = this->get_clock()->now();
                auto sent_time = rclcpp::Time(msg->data);
                auto latency = now - sent_time;
                RCLCPP_INFO(this->get_logger(), "Message latency: %f seconds", 
                           latency.seconds());
            });
    }
    
    void publish_timestamped_message() {
        auto msg = std_msgs::msg::Float64();
        msg.data = this->get_clock()->now().nanoseconds();
        publisher_->publish(msg);
    }
    
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};
```

## Common Pitfalls and Solutions

### 1. Message Size Issues

**Problem:** Large messages causing network congestion
**Solution:** Implement data compression, sampling, or splitting

### 2. Timing Issues

**Problem:** Messages arriving out of order or with incorrect timestamps
**Solution:** Use appropriate QoS settings and synchronize clocks across systems

### 3. Type Mismatch

**Problem:** Sending incorrect message types leading to runtime errors
**Solution:** Use ROS 2's type checking and validation features

### 4. Memory Issues

**Problem:** Large message queues consuming excessive memory
**Solution:** Use appropriate queue sizes and QoS settings

## Summary

Message types and communication patterns are fundamental to the design and performance of ROS 2 systems. Properly designed messages enable efficient, reliable communication between nodes in physical AI systems.

Key takeaways:
- Use standard message types when possible to promote interoperability
- Design custom messages with efficiency, extensibility, and semantic clarity in mind
- Apply appropriate QoS settings based on the communication requirements
- Consider performance implications when designing message structures
- Follow best practices for message validation and error handling

In the next chapter, we'll build on these concepts by creating our first complete ROS 2 package with publishers, subscribers, and practical examples.