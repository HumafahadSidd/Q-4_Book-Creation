# Chapter 3.2: Types of Sensors in Robotics

## Learning Objectives

By the end of this chapter, students will be able to:
- Classify different types of sensors based on their function and measurement principles
- Understand the specifications and characteristics of common robotic sensors
- Select appropriate sensors for specific robotic applications
- Analyze the trade-offs between different sensor types
- Implement sensor integration in ROS 2 systems
- Evaluate sensor performance and limitations in physical AI applications

## Introduction

Sensors are the eyes, ears, and skin of robotic systems, providing the information needed for intelligent behavior in physical environments. In robotics, sensors can be broadly classified based on what they measure: proprioceptive sensors measure the robot's own state, while exteroceptive sensors measure properties of the external environment. Each sensor type has specific characteristics, advantages, and limitations that make it suitable for particular applications.

The choice of sensors is critical for the success of physical AI systems. Different sensors provide different types of information with varying degrees of accuracy, precision, and reliability. Understanding the characteristics and capabilities of different sensor types is essential for designing effective robotic systems that can operate reliably in real-world environments.

This chapter provides a comprehensive overview of the major types of sensors used in robotics, their working principles, specifications, and applications in physical AI systems.

## Proprioceptive Sensors

Proprioceptive sensors measure the robot's own state, including joint positions, velocities, forces, and orientation. These sensors are essential for controlling the robot's movements and maintaining awareness of its own configuration.

### Joint Encoders

Joint encoders measure the angular position of robot joints and are fundamental to robot control and kinematics.

#### Types of Joint Encoders

**Incremental Encoders:**
- Measure relative position changes from a reference point
- Generate pulses as the shaft rotates
- Require a homing procedure to establish absolute position
- Advantages: Cost-effective, high resolution
- Disadvantages: Lose position information during power loss

**Absolute Encoders:**
- Provide absolute position information without homing
- Each position has a unique code
- Retain position information during power loss
- Advantages: No homing required, absolute position
- Disadvantages: More expensive, limited resolution

#### Encoder Specifications

- **Resolution**: Number of counts per revolution (e.g., 1024, 4096, 16384)
- **Accuracy**: How closely measured position matches true position
- **Repeatability**: Consistency of measurements under identical conditions
- **Maximum Speed**: Highest rotational speed the encoder can handle

#### ROS 2 Implementation

Joint encoders typically publish data through the `/joint_states` topic using the `sensor_msgs/JointState` message:

```cpp
#include "sensor_msgs/msg/joint_state.hpp"

void publish_joint_states() {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->get_clock()->now();
    msg.name = {"joint1", "joint2", "joint3"};
    msg.position = {1.57, 0.0, -0.5};  // radians
    msg.velocity = {0.1, 0.05, -0.05}; // rad/s
    msg.effort = {5.0, 3.2, 4.1};      // Nm
    
    joint_state_publisher_->publish(msg);
}
```

### Inertial Measurement Units (IMUs)

IMUs measure acceleration, angular velocity, and often magnetic field, providing information about the robot's orientation and motion.

#### Components of an IMU

**Accelerometer:**
- Measures linear acceleration along 3 axes
- Can detect tilt relative to gravity
- Used for orientation estimation when stationary

**Gyroscope:**
- Measures angular velocity around 3 axes
- Provides high-frequency orientation changes
- Subject to drift over time

**Magnetometer (optional):**
- Measures magnetic field direction
- Provides absolute orientation reference
- Susceptible to magnetic interference

#### IMU Specifications

- **Measurement Range**: Maximum acceleration/velocity measurable (e.g., ±2g to ±16g for accelerometers)
- **Resolution**: Smallest detectable change
- **Noise Density**: Noise level in the measurements
- **Bias Stability**: Drift in measurements over time
- **Bandwidth**: Frequency range of accurate measurements

#### ROS 2 Implementation

IMUs publish data using the `sensor_msgs/Imu` message:

```cpp
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

void publish_imu_data() {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "imu_link";
    
    // Orientation (quaternion: x, y, z, w)
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 1.0;
    
    // Angular velocity (rad/s)
    msg.angular_velocity.x = 0.1;
    msg.angular_velocity.y = 0.05;
    msg.angular_velocity.z = 0.02;
    
    // Linear acceleration (m/s²)
    msg.linear_acceleration.x = 0.2;
    msg.linear_acceleration.y = 0.1;
    msg.linear_acceleration.z = 9.81;  // Gravity
    
    // Covariance matrices (set to 0 if unknown)
    for (int i = 0; i < 9; ++i) {
        msg.orientation_covariance[i] = 0.0;
        msg.angular_velocity_covariance[i] = 0.0;
        msg.linear_acceleration_covariance[i] = 0.0;
    }
    
    imu_publisher_->publish(msg);
}
```

### Force/Torque Sensors

Force/torque sensors measure forces and torques applied to the robot, crucial for manipulation and safe interaction.

#### Types of Force/Torque Sensors

**Strain Gauge Sensors:**
- Measure deformation of a sensing element
- High accuracy and resolution
- Can measure multiple force/torque components

**Piezoelectric Sensors:**
- Generate electrical charge in response to force
- Good for dynamic measurements
- Require charge amplifiers

#### Specifications

- **Measurement Range**: Maximum forces/torques measurable
- **Resolution**: Smallest detectable force/torque
- **Bandwidth**: Frequency response characteristics
- **Cross-talk**: Interference between different measurement axes
- **Temperature Sensitivity**: Effect of temperature on measurements

#### ROS 2 Implementation

Force/torque sensors often publish using the `geometry_msgs/Wrench` message:

```cpp
#include "geometry_msgs/msg/wrench.hpp"

void publish_force_torque() {
    auto msg = geometry_msgs::msg::Wrench();
    
    // Forces (N)
    msg.force.x = 10.5;
    msg.force.y = 5.2;
    msg.force.z = 15.8;
    
    // Torques (Nm)
    msg.torque.x = 0.5;
    msg.torque.y = 0.3;
    msg.torque.z = 0.8;
    
    wrench_publisher_->publish(msg);
}
```

## Exteroceptive Sensors

Exteroceptive sensors measure properties of the external environment, enabling robots to perceive and interact with their surroundings.

### Cameras

Cameras provide visual information about the environment and are essential for many robotic applications.

#### Types of Cameras

**Monocular Cameras:**
- Single lens, provide 2D images
- Cost-effective, high resolution
- Require additional processing for depth information

**Stereo Cameras:**
- Two lenses, enable depth estimation
- Provide 3D information
- More expensive, computationally intensive

**RGB-D Cameras:**
- Provide color and depth information simultaneously
- Combine RGB camera with depth sensor
- Enable rich 3D scene understanding

**Thermal Cameras:**
- Detect heat signatures
- Work in low-light conditions
- Useful for detection and inspection

#### Camera Specifications

- **Resolution**: Image dimensions (e.g., 640x480, 1920x1080)
- **Frame Rate**: Images per second (e.g., 30 fps, 60 fps)
- **Field of View**: Angular extent of the scene (e.g., 60°, 90°)
- **Dynamic Range**: Range of light intensities measurable
- **Sensitivity**: Performance in low-light conditions

#### ROS 2 Implementation

Cameras publish images using the `sensor_msgs/Image` message:

```cpp
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

void publish_camera_image() {
    cv::Mat image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128)); // Example image
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = this->get_clock()->now();
    cv_image.header.frame_id = "camera_frame";
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = image;
    
    sensor_msgs::msg::Image ros_image;
    cv_image.toImageMsg(ros_image);
    
    image_publisher_->publish(ros_image);
}
```

### Range Sensors (LIDAR, Sonar, Infrared)

Range sensors measure distances to objects in the environment.

#### LIDAR (Light Detection and Ranging)

LIDAR sensors use laser light to measure distances with high precision.

**Types of LIDAR:**
- **2D LIDAR**: Single plane scanning, for navigation
- **3D LIDAR**: Multiple planes, for mapping and object detection
- **Solid-state LIDAR**: No moving parts, more robust
- **Flash LIDAR**: Illuminates entire scene at once

**Specifications:**
- **Range**: Minimum and maximum measurable distances
- **Accuracy**: Precision of distance measurements
- **Resolution**: Angular resolution of measurements
- **Field of View**: Angular extent of measurements
- **Scan Rate**: How frequently scans are performed

**ROS 2 Implementation:**
```cpp
#include "sensor_msgs/msg/laser_scan.hpp"

void publish_laser_scan() {
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "laser_frame";
    msg.angle_min = -M_PI / 2;      // -90 degrees
    msg.angle_max = M_PI / 2;       // 90 degrees
    msg.angle_increment = M_PI / 180; // 1 degree
    msg.time_increment = 0.0;
    msg.scan_time = 0.1;            // 100ms per scan
    msg.range_min = 0.1;            // 0.1m minimum range
    msg.range_max = 10.0;           // 10m maximum range
    
    // Generate example ranges
    int num_readings = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
    msg.ranges.resize(num_readings);
    for (int i = 0; i < num_readings; ++i) {
        msg.ranges[i] = 3.0 + 0.5 * sin(i * 0.1); // Example distances
    }
    
    scan_publisher_->publish(msg);
}
```

#### Ultrasonic Sensors

Ultrasonic sensors use sound waves to measure distances.

**Specifications:**
- **Range**: Typical 2cm to 4m
- **Beam Width**: Angular spread of the sound beam
- **Update Rate**: How frequently measurements are made
- **Accuracy**: Precision of distance measurements

**Advantages:**
- Cost-effective
- Work in various lighting conditions
- Good for short-range detection

**Limitations:**
- Limited accuracy
- Affected by surface properties
- Limited resolution

#### Infrared Sensors

Infrared sensors measure distances using infrared light.

**Specifications:**
- **Range**: Typically a few cm to several meters
- **Resolution**: Distance accuracy
- **Response Time**: How quickly measurements are made

**Advantages:**
- Small size
- Low power consumption
- Good for proximity detection

**Limitations:**
- Affected by ambient light
- Limited range
- Affected by surface reflectivity

### Tactile Sensors

Tactile sensors provide information about touch, pressure, and texture, essential for dexterous manipulation.

#### Types of Tactile Sensors

**Force Sensing Resistors (FSRs):**
- Change resistance with applied force
- Simple and cost-effective
- Provide force magnitude only

**Piezoelectric Sensors:**
- Generate voltage with applied force
- Good for dynamic measurements
- Self-generating (no external power needed)

**Capacitive Sensors:**
- Measure changes in capacitance
- Can detect proximity without contact
- Good spatial resolution

**Optical Tactile Sensors:**
- Use light to detect surface deformation
- High resolution and accuracy
- Complex but very informative

#### Specifications

- **Resolution**: Smallest detectable force
- **Range**: Maximum measurable force
- **Response Time**: How quickly the sensor responds
- **Spatial Resolution**: For array sensors, number of sensing elements

#### ROS 2 Implementation

Tactile sensors often use custom messages or the `sensor_msgs/JointState` message:

```cpp
#include "std_msgs/msg/float32_multi_array.hpp"

void publish_tactile_data() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    msg.layout.dim[0].size = 16;  // 16 tactile sensors
    msg.layout.dim[0].stride = 1;
    msg.layout.data_offset = 0;
    
    // Example tactile readings
    msg.data = {0.1, 0.3, 0.2, 0.5, 0.8, 1.2, 0.9, 0.4, 
                0.2, 0.6, 0.7, 0.3, 0.1, 0.4, 0.6, 0.9};
    
    tactile_publisher_->publish(msg);
}
```

## Sensor Fusion

Sensor fusion combines data from multiple sensors to provide more accurate and reliable information than any single sensor could provide.

### Fusion Techniques

**Kalman Filtering:**
- Optimal fusion for linear systems with Gaussian noise
- Recursively estimates state from noisy measurements
- Widely used for state estimation

**Particle Filtering:**
- Handles non-linear systems and non-Gaussian noise
- Represents probability distributions with samples
- Computationally more intensive

**Bayesian Networks:**
- Represents probabilistic relationships between variables
- Handles uncertainty in complex systems
- Good for reasoning under uncertainty

### Common Fusion Examples

**IMU + Visual Odometry:**
- Combines inertial and visual information
- Provides robust pose estimation
- Compensates for individual sensor limitations

**LIDAR + Camera:**
- Combines geometric and visual information
- Enables rich environmental understanding
- Supports both navigation and recognition

**Multiple Cameras:**
- Stereo vision for depth estimation
- Multi-view geometry for 3D reconstruction
- Redundancy for reliability

## Sensor Selection Guidelines

### Factors to Consider

**Task Requirements:**
- What information is needed?
- What accuracy is required?
- What is the operating environment?

**Technical Specifications:**
- Range, accuracy, resolution
- Update rate, latency
- Power consumption, size

**Environmental Conditions:**
- Lighting conditions
- Temperature range
- Dust, water, vibration

**Cost and Complexity:**
- Budget constraints
- Integration complexity
- Maintenance requirements

### Selection Process

1. **Define Requirements:** What information does the robot need?
2. **Identify Constraints:** Environmental, budget, size, power
3. **Research Options:** Available sensors that meet requirements
4. **Evaluate Trade-offs:** Compare options based on criteria
5. **Prototype and Test:** Validate selected sensors in real conditions

## Sensor Integration in ROS 2

### Sensor Message Types

ROS 2 provides standard message types for common sensors:

- `sensor_msgs/Image`: Camera images
- `sensor_msgs/LaserScan`: 2D LIDAR scans
- `sensor_msgs/PointCloud2`: 3D point cloud data
- `sensor_msgs/Imu`: Inertial measurement data
- `sensor_msgs/JointState`: Joint position/velocity/effort
- `sensor_msgs/BatteryState`: Battery information

### Sensor Drivers

Most sensors have ROS 2 drivers that publish standardized messages:

```cpp
// Example sensor driver structure
class SensorDriver : public rclcpp::Node
{
public:
    SensorDriver() : Node("sensor_driver")
    {
        // Initialize hardware interface
        init_hardware();
        
        // Create publisher for sensor data
        sensor_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        
        // Create timer for periodic data publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorDriver::read_sensor, this));
    }

private:
    void read_sensor()
    {
        // Read from hardware
        auto data = read_from_hardware();
        
        // Convert to ROS message
        auto msg = convert_to_ros_message(data);
        
        // Publish message
        sensor_publisher_->publish(msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sensor_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Sensor Calibration

Proper calibration is essential for accurate sensor measurements:

### Camera Calibration

```cpp
// Example camera calibration using OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

// Calibrate camera intrinsics
cv::calibrateCamera(object_points, image_points, image_size,
                   camera_matrix, distortion_coeffs, rvecs, tvecs);
```

### LIDAR Calibration

- Extrinsic calibration: Position and orientation relative to robot
- Intrinsic calibration: Internal parameters of the sensor
- Alignment with other sensors

### IMU Calibration

- Bias calibration: Remove systematic errors
- Scale factor calibration: Correct for scaling errors
- Alignment calibration: Correct for axis misalignment

## Sensor Data Processing

### Filtering and Preprocessing

Raw sensor data often requires filtering and preprocessing:

```cpp
// Example: Simple moving average filter for sensor data
class SensorFilter
{
public:
    SensorFilter(int window_size) : window_size_(window_size) {}
    
    double update(double new_value)
    {
        if (values_.size() >= window_size_) {
            values_.pop_front();
        }
        values_.push_back(new_value);
        
        double sum = 0.0;
        for (double val : values_) {
            sum += val;
        }
        return sum / values_.size();
    }

private:
    int window_size_;
    std::deque<double> values_;
};
```

### Outlier Detection

```cpp
// Example: Statistical outlier detection
bool is_outlier(double value, double mean, double std_dev, double threshold = 2.0)
{
    return std::abs(value - mean) > threshold * std_dev;
}
```

## Summary

Sensors are fundamental to physical AI systems, providing the information needed for intelligent behavior in real-world environments. Different sensor types provide different types of information with varying characteristics, advantages, and limitations.

Key concepts include:
- Proprioceptive sensors measure the robot's own state (encoders, IMUs, force sensors)
- Exteroceptive sensors measure the external environment (cameras, LIDAR, tactile sensors)
- Sensor specifications determine suitability for specific applications
- Sensor fusion combines multiple sensors for improved performance
- Proper calibration and integration are essential for reliable operation

The selection and integration of appropriate sensors is critical for the success of physical AI systems. In the next chapter, we'll explore sensor data processing and filtering techniques.