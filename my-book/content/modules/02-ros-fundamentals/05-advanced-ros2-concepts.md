# Chapter 2.5: Advanced ROS 2 Concepts - Launch Files, Parameters, and Tools

## Learning Objectives

By the end of this chapter, students will be able to:
- Create complex launch files for managing multi-node systems
- Implement parameter management for configurable robotic systems
- Use advanced ROS 2 tools for system monitoring and debugging
- Apply lifecycle management for complex robotic systems
- Integrate external tools and services with ROS 2 systems
- Optimize system performance using advanced techniques

## Introduction

In the previous chapters, we covered the fundamentals of ROS 2 nodes, topics, services, and basic package creation. Now we'll explore advanced concepts that are essential for building production-ready physical AI systems. These concepts include launch files for managing complex multi-node systems, parameter management for configurable behavior, and advanced tools for monitoring, debugging, and optimizing robotic applications.

Advanced ROS 2 concepts enable the development of sophisticated robotic systems that can adapt to different environments, handle complex initialization procedures, and scale to multiple robots. Understanding these concepts is crucial for developing physical AI systems that operate reliably in real-world environments.

## Advanced Launch File Concepts

### Launch File Structure

Launch files in ROS 2 are Python scripts that define how to launch a collection of nodes and other processes. They use the launch system to manage the lifecycle of these processes.

Basic launch file structure:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    param_file = LaunchConfiguration('param_file')
    
    # Create nodes
    node = Node(
        package='package_name',
        executable='executable_name',
        name='node_name',
        parameters=[param_file],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value='path/to/default/params.yaml',
            description='Path to parameter file'
        ),
        node
    ])
```

### Complex Launch File with Multiple Nodes

Here's a more complex example showing multiple nodes with dependencies:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=['path/to/robot_description.yaml']
    )
    
    # Spawn robot in Gazebo after Gazebo starts
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )
    
    # Register event handler to spawn robot after Gazebo starts
    spawn_entity_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[spawn_entity]
        )
    )
    
    # Navigation nodes
    nav_bringup = Node(
        package='nav2_bringup',
        executable='nav2_bringup',
        parameters=['path/to/nav_params.yaml']
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity_after_gazebo,
        nav_bringup
    ])
```

### Launch File Parameters and Conditional Launching

Launch files can accept parameters and conditionally launch nodes:

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def launch_setup(context: LaunchContext):
    # Get launch configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_vision = LaunchConfiguration('enable_vision')
    
    # Create nodes conditionally
    nodes = []
    
    # Always launch the base controller
    base_controller = Node(
        package='my_robot_control',
        executable='base_controller',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    nodes.append(base_controller)
    
    # Conditionally launch vision node
    vision_node = Node(
        package='my_vision',
        executable='vision_node',
        condition=IfCondition(enable_vision)
    )
    nodes.append(vision_node)
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'enable_vision',
            default_value='true',
            description='Enable vision processing'
        ),
        
        # Use OpaqueFunction to access launch configurations
        OpaqueFunction(function=launch_setup)
    ])
```

## Parameter Management

### Parameter Declaration and Usage

Parameters in ROS 2 provide a way to configure node behavior without recompiling. Nodes can declare parameters and access their values at runtime.

**Parameter declaration in C++:**

```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterExampleNode : public rclcpp::Node
{
public:
    ParameterExampleNode() : Node("parameter_example_node")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("robot_name", "default_robot");
        this->declare_parameter<double>("control_frequency", 50.0);
        this->declare_parameter<std::vector<double>>("safety_limits", {1.0, 2.0, 3.0});
        this->declare_parameter<bool>("enable_logging", true);
        
        // Get parameter values
        robot_name_ = this->get_parameter("robot_name").as_string();
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        safety_limits_ = this->get_parameter("safety_limits").as_double_array();
        enable_logging_ = this->get_parameter("enable_logging").as_bool();
        
        // Set up a parameter callback for dynamic reconfiguration
        parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterExampleNode::parametersCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), 
            "Initialized with robot_name: %s, control_frequency: %.2f", 
            robot_name_.c_str(), control_frequency_);
    }

private:
    rclcpp::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackType parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "control_frequency" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (parameter.as_double() > 0.0 && parameter.as_double() <= 1000.0) {
                    control_frequency_ = parameter.as_double();
                    RCLCPP_INFO(this->get_logger(), "Control frequency updated to: %.2f", control_frequency_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid control frequency value: %.2f", parameter.as_double());
                    return rclcpp::OnSetParametersCallbackInterface::PreReturnCode::FAILURE;
                }
            }
        }
        return rclcpp::OnSetParametersCallbackInterface::PreReturnCode::SUCCESS;
    }
    
    std::string robot_name_;
    double control_frequency_;
    std::vector<double> safety_limits_;
    bool enable_logging_;
};
```

### Parameter Files

Parameters can be organized in YAML files for easier management:

**config/robot_params.yaml:**
```yaml
/**:  # Applies to all nodes
  ros__parameters:
    use_sim_time: false
    log_level: "info"

robot_controller:
  ros__parameters:
    max_velocity: 1.0
    acceleration_limit: 2.0
    safety_margin: 0.5

sensor_processor:
  ros__parameters:
    detection_range: 3.0
    update_rate: 10.0
    confidence_threshold: 0.8

navigation_system:
  ros__parameters:
    planner_frequency: 5.0
    controller_frequency: 20.0
    recovery_enabled: true
    max_vel_x: 0.5
    min_vel_x: 0.2
    max_vel_theta: 1.0
    min_vel_theta: 0.4
    min_in_place_vel_theta: 0.4
    xy_goal_tolerance: 0.25
    yaw_goal_tolerance: 0.1
    rot_stopped_vel: 0.01
    trans_stopped_vel: 0.01
```

### Loading Parameters from Files

Parameters can be loaded from YAML files when launching nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to parameter file
    param_file = PathJoinSubstitution([
        FindPackageShare('my_robot_package'),
        'config',
        'robot_params.yaml'
    ])
    
    # Node with parameter file
    robot_controller = Node(
        package='my_robot_package',
        executable='robot_controller',
        parameters=[param_file],
        output='screen'
    )
    
    return LaunchDescription([
        robot_controller
    ])
```

### Dynamic Parameter Reconfiguration

Parameters can be changed at runtime using command-line tools:

```bash
# Set a parameter
ros2 param set /node_name parameter_name parameter_value

# Get a parameter
ros2 param get /node_name parameter_name

# List all parameters for a node
ros2 param list /node_name

# Get all parameter values for a node
ros2 param dump /node_name
```

## Advanced ROS 2 Tools

### System Monitoring Tools

ROS 2 provides several command-line tools for monitoring and debugging systems:

**ros2 node:**
```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info /node_name

# Get a node's parameters
ros2 param list /node_name
```

**ros2 topic:**
```bash
# List all active topics
ros2 topic list

# Get information about a topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name

# Show message type
ros2 topic type /topic_name

# Publish a message to a topic
ros2 topic pub /topic_name message_type "field1: value1; field2: value2"

# Show topic statistics
ros2 topic hz /topic_name  # Shows frequency
ros2 topic bw /topic_name  # Shows bandwidth
```

**ros2 service:**
```bash
# List all available services
ros2 service list

# Call a service
ros2 service call /service_name service_type "{request_field: value}"

# Show service type
ros2 service type /service_name
```

**ros2 action:**
```bash
# List all available actions
ros2 action list

# Send an action goal
ros2 action send_goal /action_name action_type "{goal_field: value}"

# Show action type
ros2 action type /action_name
```

### Advanced Debugging with ros2 doctor

The `ros2 doctor` tool provides system health checks:

```bash
# Run all checks
ros2 doctor

# Run specific checks
ros2 doctor --report network

# Check specific topics
ros2 doctor --topic /scan
```

### Profiling and Performance Analysis

For performance analysis, ROS 2 provides the `ros2 trace` tool:

```bash
# Trace the system for 10 seconds
ros2 trace --duration 10

# Trace specific nodes
ros2 trace --duration 10 --events-callbacks --events-composition /node_name
```

## Lifecycle Management

ROS 2 provides lifecycle nodes for managing complex initialization, configuration, and shutdown procedures:

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class LifecycleController : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleController() : rclcpp_lifecycle::LifecycleNode("lifecycle_controller")
    {
        RCLCPP_INFO(get_logger(), "Lifecycle controller initialized");
    }

protected:
    // Configure state: Initialize resources
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        
        // Initialize publishers, subscribers, services, etc.
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sensor_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LifecycleController::sensor_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Lifecycle controller configured");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Activate state: Start operations
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        
        // Activate publishers and subscribers
        cmd_publisher_->on_activate();
        
        // Start control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LifecycleController::control_loop, this));
        
        RCLCPP_INFO(get_logger(), "Lifecycle controller activated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Deactivate state: Stop operations
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        
        // Stop control loop
        control_timer_.reset();
        
        // Deactivate publishers
        cmd_publisher_->on_deactivate();
        
        RCLCPP_INFO(get_logger(), "Lifecycle controller deactivated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Cleanup state: Release resources
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        
        // Reset publishers, subscribers, etc.
        cmd_publisher_.reset();
        sensor_subscriber_.reset();
        control_timer_.reset();
        
        RCLCPP_INFO(get_logger(), "Lifecycle controller cleaned up");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Shutdown state: Final cleanup
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "Lifecycle controller shutting down");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Error state: Handle errors
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State & state)
    {
        (void)state;
        RCLCPP_ERROR(get_logger(), "Lifecycle controller error state");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process sensor data
        last_scan_ = msg;
    }
    
    void control_loop()
    {
        if (last_scan_ != nullptr) {
            // Implement control logic based on sensor data
            auto cmd_msg = geometry_msgs::msg::Twist();
            
            // Example: Simple obstacle avoidance
            float min_range = *std::min_element(last_scan_->ranges.begin(), last_scan_->ranges.end());
            
            if (min_range < 1.0) {
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.5;  // Turn
            } else {
                cmd_msg.linear.x = 0.5;   // Move forward
                cmd_msg.angular.z = 0.0;
            }
            
            cmd_publisher_->publish(cmd_msg);
        }
    }
    
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sensor_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};
```

### Managing Lifecycle Nodes

Lifecycle nodes can be managed using command-line tools:

```bash
# List lifecycle nodes
ros2 lifecycle list /node_name

# Get current state
ros2 lifecycle get /node_name

# Change state
ros2 lifecycle configure /node_name
ros2 lifecycle activate /node_name
ros2 lifecycle deactivate /node_name
ros2 lifecycle cleanup /node_name
ros2 lifecycle shutdown /node_name
```

## Integration with External Tools

### Using ROS 2 with Docker

ROS 2 systems can be containerized using Docker for consistent deployment:

**Dockerfile:**
```dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ws
COPY . src/
RUN source /opt/ros/humble/setup.bash && \
    colcon build && \
    rm -rf build/ log/

# Source setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
```

### Using ROS 2 with Simulation

ROS 2 integrates with various simulation environments:

**Gazebo Integration:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Launch Gazebo with a specific world
    world_file = os.path.join(
        get_package_share_directory('my_robot_gazebo'),
        'worlds',
        'my_world.sdf'
    )
    
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=['path/to/robot_description.yaml']
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0',
            '-y', '0', 
            '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
```

## Performance Optimization Techniques

### Efficient Message Handling

Optimize message handling for performance:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class OptimizedNode : public rclcpp::Node
{
public:
    OptimizedNode() : Node("optimized_node")
    {
        // Use intra-process communication when possible
        rclcpp::QoS qos(10);
        qos.intra_process_comm(rclcpp::IntraProcessSetting::Enable);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            qos,
            std::bind(&OptimizedNode::scan_callback, this, std::placeholders::_1));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Pre-allocate vectors to avoid memory allocation in callback
        if (ranges_cache_.size() != msg->ranges.size()) {
            ranges_cache_.resize(msg->ranges.size());
        }
        
        // Process data efficiently
        std::copy(msg->ranges.begin(), msg->ranges.end(), ranges_cache_.begin());
        
        // Find minimum range efficiently
        auto min_it = std::min_element(ranges_cache_.begin(), ranges_cache_.end());
        if (min_it != ranges_cache_.end()) {
            min_range_ = *min_it;
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::vector<float> ranges_cache_;
    float min_range_{std::numeric_limits<float>::max()};
};
```

### Quality of Service (QoS) Optimization

Choose appropriate QoS settings for your application:

```cpp
// For sensor data (real-time, lose some data is okay)
auto sensor_qos = rclcpp::QoS(5)  // small history
    .best_effort()
    .durability_volatile();

// For critical control commands (must be delivered)
auto cmd_qos = rclcpp::QoS(1)  // minimal history
    .reliable()
    .durability_volatile();

// For state information (new nodes should get latest)
auto state_qos = rclcpp::QoS(1)
    .reliable()
    .transient_local();
```

### Multi-threading in ROS 2

Use multi-threading for CPU-intensive operations:

```cpp
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <future>

class MultiThreadedNode : public rclcpp::Node
{
public:
    MultiThreadedNode() : Node("multi_threaded_node")
    {
        // Create multi-threaded executor
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        
        // Create publisher and subscription
        publisher_ = this->create_publisher<std_msgs::msg::String>("/output", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/input", 10,
            std::bind(&MultiThreadedNode::input_callback, this, std::placeholders::_1));
    }
    
    void process_data() {
        executor_->add_node(this->get_node_base_interface());
        executor_->spin();
    }

private:
    void input_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // For CPU-intensive processing, use a separate thread
        std::thread processing_thread(&MultiThreadedNode::intensive_processing, this, msg->data);
        processing_thread.detach();  // Detach to avoid blocking
    }
    
    void intensive_processing(const std::string& input_data)
    {
        // Simulate intensive processing
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Publish result
        auto result_msg = std_msgs::msg::String();
        result_msg.data = "Processed: " + input_data;
        publisher_->publish(result_msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};
```

## Best Practices for Advanced ROS 2 Development

### 1. System Design
- Use launch files to manage complex multi-node systems
- Implement parameter management for configurable behavior
- Use lifecycle nodes for complex initialization procedures
- Apply appropriate QoS settings for different data types

### 2. Performance
- Pre-allocate memory in callbacks to avoid allocation overhead
- Use intra-process communication when nodes are in the same process
- Choose appropriate queue sizes based on system requirements
- Monitor system performance using ROS 2 tools

### 3. Debugging and Monitoring
- Use structured logging with appropriate levels
- Implement parameter validation and error handling
- Use launch file arguments for different configurations
- Monitor system health with `ros2 doctor`

### 4. Maintainability
- Organize parameters in YAML files for easier management
- Use descriptive names for nodes, topics, and parameters
- Document launch files and parameter meanings
- Implement graceful error handling and recovery

## Summary

Advanced ROS 2 concepts provide the tools and techniques needed to build production-ready physical AI systems. Launch files enable the management of complex multi-node systems, parameter management allows for configurable behavior, and advanced tools facilitate monitoring and debugging.

Key takeaways:
- Launch files provide a powerful way to manage complex robotic systems
- Parameter management enables configurable behavior without recompilation
- Advanced tools help monitor, debug, and optimize ROS 2 systems
- Lifecycle management handles complex initialization and shutdown procedures
- Performance optimization techniques ensure efficient operation

These advanced concepts are essential for developing physical AI systems that operate reliably in real-world environments. In the next module, we'll explore sensorimotor integration and perception-action loops, building on these advanced ROS 2 concepts.