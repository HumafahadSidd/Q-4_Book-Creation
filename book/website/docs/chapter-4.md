---
sidebar_position: 5
---

# Chapter 4: ROS 2 with Python

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Set up a ROS 2 Python development environment
- Create and run basic ROS 2 nodes using rclpy
- Implement publishers and subscribers
- Work with parameters and launch files
- Understand the structure of ROS 2 Python packages

## 4.1 Setting Up ROS 2 with Python

ROS 2 provides Python support through the rclpy library, which is a Python client library for ROS 2. To get started, ensure you have ROS 2 installed (Humble Hawksbill or later) and source the ROS 2 setup script.

```bash
source /opt/ros/humble/setup.bash
```

## 4.2 Creating Your First ROS 2 Node

A basic ROS 2 node in Python follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.3 Publishers and Subscribers

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## 4.4 Working with Parameters

ROS 2 allows nodes to have configurable parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_param', 'default_value')
        
        # Get parameter value
        param_value = self.get_parameter('my_param').value
        self.get_logger().info(f'Parameter value: {param_value}')
```

## 4.5 Launch Files

Launch files allow you to start multiple nodes with a single command. Create a launch directory in your package and add a Python launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name',
            parameters=[
                {'param_name': 'param_value'}
            ]
        )
    ])
```

## 4.6 Chapter Summary

In this chapter, we explored ROS 2 with Python, covering how to create nodes, implement publishers and subscribers, work with parameters, and use launch files. These concepts form the foundation for building more complex robotic applications.

In the next chapter, we'll learn about URDF (Unified Robot Description Format) and how to model humanoid robots.

## Exercises

1. Create a simple publisher that publishes sensor data (e.g., temperature readings).
2. Create a subscriber that processes the sensor data and logs it.
3. Implement a node with a configurable parameter for the publishing rate.

---