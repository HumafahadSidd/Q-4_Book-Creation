# Chapter 2.2: Nodes, Topics, Services, and Actions

## Learning Objectives

By the end of this chapter, students will be able to:
- Create and manage ROS 2 nodes in C++ and Python
- Implement publish-subscribe communication using topics
- Develop request-response communication using services
- Design goal-oriented communication using actions
- Select the appropriate communication pattern for specific use cases
- Debug and monitor communication between nodes

## Introduction

In the previous chapter, we introduced the core components of the ROS 2 architecture. Now we'll dive deeper into the four fundamental communication mechanisms that enable nodes to interact with each other: nodes, topics, services, and actions. These components form the backbone of any ROS 2 application and are essential for building physical AI systems that can perceive, process, and act in the real world.

Understanding how to effectively use these communication patterns is crucial for developing robust robotic systems. Each pattern serves specific purposes and has its own advantages and limitations. By mastering these concepts, you'll be able to design and implement complex robotic behaviors that embody the principles of physical AI.

## Nodes: The Fundamental Computational Units

### Node Definition

A node is an executable that uses ROS 2 to communicate with other nodes. It contains the computational processes that perform specific functions within the robotic system. Nodes are the smallest unit of computation that can be individually managed, monitored, and debugged.

### Creating Nodes in C++

Let's implement a simple node that simulates a basic sensor:

```cpp
// sensor_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SensorNode : public rclcpp::Node
{
public:
    SensorNode() : Node("sensor_node")
    {
        // Create a publisher for laser scan messages
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 10);
        
        // Create a timer to periodically publish sensor data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Publish every 100ms
            std::bind(&SensorNode::publish_scan, this));
        
        RCLCPP_INFO(this->get_logger(), "Sensor node initialized");
    }

private:
    void publish_scan()
    {
        auto message = sensor_msgs::msg::LaserScan();
        
        // Set up basic laser scan parameters
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "laser_frame";
        message.angle_min = -1.57;  // -90 degrees in radians
        message.angle_max = 1.57;   // 90 degrees in radians
        message.angle_increment = 0.01745; // ~1 degree
        message.time_increment = 0.0;
        message.scan_time = 0.1;  // 100ms per scan
        message.range_min = 0.1;
        message.range_max = 10.0;
        
        // Generate some sample data
        size_t num_readings = std::ceil((message.angle_max - message.angle_min) 
                                      / message.angle_increment);
        message.ranges.resize(num_readings);
        
        // Fill with sample data (in a real sensor, this would come from hardware)
        for (size_t i = 0; i < num_readings; ++i) {
            message.ranges[i] = 2.0 + 0.5 * sin(i * 0.1); // Simulated distance readings
        }
        
        publisher_->publish(message);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Creating Nodes in Python

The same sensor node implemented in Python:

```python
#!/usr/bin/env python3
# sensor_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Create a publisher for laser scan messages
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Create a timer to periodically publish sensor data
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_scan)
        
        self.get_logger().info('Sensor node initialized')

    def publish_scan(self):
        msg = LaserScan()
        
        # Set up basic laser scan parameters
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -1.57  # -90 degrees in radians
        msg.angle_max = 1.57   # 90 degrees in radians
        msg.angle_increment = 0.01745  # ~1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 100ms per scan
        msg.range_min = 0.1
        msg.range_max = 10.0
        
        # Generate some sample data
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [0.0] * num_readings
        
        # Fill with sample data (in a real sensor, this would come from hardware)
        for i in range(num_readings):
            msg.ranges[i] = 2.0 + 0.5 * math.sin(i * 0.1)  # Simulated distance readings
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle Management

ROS 2 provides lifecycle nodes for managing complex initialization, configuration, and shutdown procedures:

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class LifecycleSensorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleSensorNode() : rclcpp_lifecycle::LifecycleNode("lifecycle_sensor_node")
    {
        RCLCPP_INFO(get_logger(), "Lifecycle sensor node initialized");
    }

protected:
    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        // Initialize resources, open connections, etc.
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        RCLCPP_INFO(get_logger(), "Lifecycle sensor node configured");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        publisher_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LifecycleSensorNode::publish_scan, this));
        RCLCPP_INFO(get_logger(), "Lifecycle sensor node activated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        timer_.reset();
        publisher_->on_deactivate();
        RCLCPP_INFO(get_logger(), "Lifecycle sensor node deactivated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        publisher_.reset();
        timer_.reset();
        RCLCPP_INFO(get_logger(), "Lifecycle sensor node cleaned up");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void publish_scan()
    {
        // Implementation similar to the basic sensor node
    }

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

## Topics: Publish-Subscribe Communication

### Topic Fundamentals

Topics implement a publish-subscribe communication pattern where publishers send messages to topics and subscribers receive messages from topics they're interested in. This pattern enables loose coupling between nodes and asynchronous communication.

### Creating Publishers

A publisher is created within a node to send messages to a topic:

```cpp
// Create a publisher with default QoS
auto publisher = this->create_publisher<std_msgs::msg::String>("/topic_name", 10);

// Create a publisher with custom QoS
rclcpp::QoS qos(10);  // history depth of 10
qos.reliable();       // ensure reliable delivery
qos.durability_volatile();  // don't store messages for late joiners
auto publisher = this->create_publisher<std_msgs::msg::String>("/topic_name", qos);
```

### Creating Subscribers

A subscriber receives messages from a topic:

```cpp
// Create a subscriber with a callback function
auto subscription = this->create_subscription<std_msgs::msg::String>(
    "/topic_name",
    10,  // queue size
    [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    });

// Or using a separate callback function
auto subscription = this->create_subscription<std_msgs::msg::String>(
    "/topic_name",
    10,
    std::bind(&MyNode::topic_callback, this, std::placeholders::_1));

void MyNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
}
```

### Complete Publisher-Subscriber Example

Here's a complete example showing both a publisher and subscriber:

**Publisher Node:**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PublisherNode::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS 2 - " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_ = 0;
};
```

**Subscriber Node:**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/chatter",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

## Services: Request-Response Communication

### Service Fundamentals

Services implement a request-response communication pattern where a client sends a request and waits for a response from a service server. This pattern is synchronous and useful for operations that require confirmation or return specific results.

### Creating Services

First, you need to define a service interface. Create a file called `AddTwoInts.srv`:

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### Service Server Implementation

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"  // Use standard service

class ServiceServerNode : public rclcpp::Node
{
public:
    ServiceServerNode() : Node("service_server_node")
    {
        // Create the service server
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "/add_two_ints",
            [this](const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                   example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
                response->sum = request->a + request->b;
                RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld, b: %ld", 
                           request->a, request->b);
                RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
            });
    }

private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

### Service Client Implementation

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClientNode : public rclcpp::Node
{
public:
    ServiceClientNode() : Node("service_client_node")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("/add_two_ints");
        
        // Wait for service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        // Call the service
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;
        
        auto future = client_->async_send_request(request);
        // Handle the response when it's ready
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld", result->sum);
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
```

## Actions: Goal-Oriented Communication

### Action Fundamentals

Actions provide a communication pattern for long-running, goal-oriented tasks. They combine features of topics and services, providing feedback during execution and reporting results upon completion.

### Action Definition

Create an action definition file called `Fibonacci.action`:

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

### Action Server Implementation

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionServer : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    FibonacciActionServer() : Node("fibonacci_action_server")
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        using namespace std::placeholders;
        // This needs to return quickly, so spin up a new thread
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        
        // Create feedback and result messages
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        // Start executing the action
        auto sequence = std::vector<int32_t>{};
        sequence.push_back(0);
        sequence.push_back(1);

        auto goal = goal_handle->get_goal();
        auto order = static_cast<size_t>(goal->order);

        // Send feedback periodically
        for (size_t i = 1; (i < order) && (rclcpp::ok()); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                return;
            }

            // Update sequence
            sequence.push_back(sequence[i] + sequence[i - 1]);

            // Publish feedback
            feedback->sequence = sequence;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Publishing feedback");

            // Sleep to simulate work
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};
```

### Action Client Implementation

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionClient : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;

    FibonacciActionClient() : Node("fibonacci_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FibonacciActionClient::send_goal, this));
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&FibonacciActionClient::result_callback, this, _1);

        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Next number in sequence received: %" PRId32,
            feedback->sequence.back());
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Result received:");
        for (auto number : result.result->sequence) {
            RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
        }

        rclcpp::shutdown();
    }
};
```

## Communication Pattern Selection Guide

### When to Use Each Pattern

| Pattern | Use Case | Characteristics |
|---------|----------|----------------|
| **Topics** | Continuous data streams, sensor data, state updates | Asynchronous, one-to-many, real-time |
| **Services** | Single requests with immediate responses | Synchronous, one-to-one, transactional |
| **Actions** | Long-running tasks with feedback | Goal-oriented, cancellable, status reporting |

### Examples in Physical AI Systems

**Topics:**
- Sensor data (camera images, LIDAR scans, IMU readings)
- Robot state (joint positions, odometry, battery level)
- Control commands (velocity commands, joint trajectories)

**Services:**
- Map saving/loading
- Calibration procedures
- System configuration changes
- Emergency stop activation

**Actions:**
- Navigation to goal positions
- Manipulation tasks
- Calibration sequences
- Complex motion planning

## Debugging and Monitoring

### ROS 2 Command Line Tools

**Node Monitoring:**
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info /node_name

# Echo the parameters of a node
ros2 param list /node_name
```

**Topic Monitoring:**
```bash
# List all active topics
ros2 topic list

# Get information about a topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name

# Show the type of messages on a topic
ros2 topic type /topic_name
```

**Service Monitoring:**
```bash
# List all available services
ros2 service list

# Call a service
ros2 service call /service_name service_type "{request_field: value}"

# Show service type
ros2 service type /service_name
```

**Action Monitoring:**
```bash
# List all available actions
ros2 action list

# Send an action goal
ros2 action send_goal /action_name action_type "{goal_field: value}"
```

### Programming-Based Monitoring

```cpp
// Check if a service is available
if (client_->service_is_ready()) {
    // Proceed with service call
}

// Check if a topic has publishers
if (publisher_->get_subscription_count() > 0) {
    // Safe to publish
}
```

## Best Practices

### Node Design
1. **Single Responsibility**: Each node should perform a single, well-defined function
2. **Error Handling**: Implement proper error handling and recovery mechanisms
3. **Resource Management**: Properly initialize and clean up resources
4. **Logging**: Use appropriate logging levels for debugging and monitoring

### Communication Design
1. **Appropriate Pattern**: Choose the right communication pattern for your use case
2. **Message Efficiency**: Design efficient message structures to minimize network overhead
3. **QoS Selection**: Choose appropriate QoS settings for your application's requirements
4. **Naming Conventions**: Use consistent and descriptive names for topics, services, and actions

### Performance Considerations
1. **Message Rate**: Balance update frequency with computational load
2. **Queue Sizes**: Set appropriate queue sizes to avoid message loss
3. **Network Topology**: Consider bandwidth and latency in distributed systems
4. **Resource Monitoring**: Monitor CPU and memory usage across nodes

## Summary

Nodes, topics, services, and actions form the core communication mechanisms in ROS 2. Understanding how to effectively use each pattern is essential for developing robust physical AI systems. 

- **Nodes** are the fundamental computational units that perform specific functions
- **Topics** enable asynchronous, publish-subscribe communication for continuous data streams
- **Services** provide synchronous, request-response communication for transactional operations
- **Actions** support goal-oriented communication for long-running tasks with feedback

The choice of communication pattern depends on your specific use case, with topics being ideal for continuous data, services for immediate requests, and actions for complex, long-running operations. In the next chapter, we'll explore message types and communication patterns in more detail.