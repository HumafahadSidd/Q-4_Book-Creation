---
sidebar_position: 8
---

# Chapter 7: Sensor Simulation

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Understand different types of sensors used in robotics
- Configure camera, LIDAR, and IMU sensors in Gazebo
- Implement sensor noise and realism parameters
- Process simulated sensor data in ROS 2
- Evaluate sensor performance in simulation

## 7.1 Introduction to Robot Sensors

Sensors are the eyes and ears of robots, providing crucial information about the environment and the robot's state. In physical AI systems, sensors enable perception and allow robots to interact with the world effectively.

Key sensor categories:
- Proprioceptive: Measure internal robot state (joint angles, IMU)
- Exteroceptive: Measure external environment (cameras, LIDAR, sonar)
- Tactile: Measure contact and force (force/torque sensors, tactile arrays)

## 7.2 Camera Sensors

Camera sensors provide visual information about the environment. In Gazebo, camera sensors can be configured with various parameters:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <topic_name>image_raw</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

Camera parameters include:
- Resolution: Image width and height
- Field of view: Angular extent of the scene
- Frame rate: How often images are captured
- Noise: Simulation of real sensor noise

## 7.3 LIDAR Sensors

LIDAR (Light Detection and Ranging) sensors provide distance measurements by emitting laser beams and measuring the time for reflections. They're essential for mapping and navigation:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
      <topic_name>scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

LIDAR parameters include:
- Range: Minimum and maximum detection distances
- Resolution: Angular resolution of measurements
- Field of view: Angular extent of scanning
- Update rate: Frequency of measurements

## 7.4 IMU Sensors

IMU (Inertial Measurement Unit) sensors measure acceleration and angular velocity. They're crucial for balance and navigation:

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## 7.5 Sensor Noise and Realism

Real sensors have noise and imperfections that should be simulated:
- Gaussian noise: Random variations in measurements
- Bias: Systematic errors in measurements
- Drift: Slow changes in sensor characteristics over time

Adding realistic noise helps ensure algorithms work with real-world sensor data.

## 7.6 Processing Sensor Data in ROS 2

ROS 2 provides standardized message types for sensor data:
- `sensor_msgs/Image`: Camera images
- `sensor_msgs/LaserScan`: LIDAR data
- `sensor_msgs/Imu`: IMU measurements
- `sensor_msgs/JointState`: Joint positions and velocities

Example of processing camera data:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Process the image (e.g., detect objects)
        # ... processing code ...
        
        # Display the image
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)
```

## 7.7 Chapter Summary

In this chapter, we explored sensor simulation in Gazebo, covering cameras, LIDAR, and IMU sensors. We learned how to configure these sensors with realistic parameters and process their data in ROS 2.

In the next chapter, we'll look at Unity for visualization and human-robot interaction.

## Exercises

1. Add a camera sensor to your robot model and visualize the output.
2. Configure a LIDAR sensor and process the scan data in a ROS 2 node.
3. Implement a simple object detection algorithm using camera data.

---