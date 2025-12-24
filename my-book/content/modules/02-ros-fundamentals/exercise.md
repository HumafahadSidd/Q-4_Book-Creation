# Module 2 Exercise: ROS 2 Fundamentals

## Learning Objectives

After completing this exercise, you will be able to:
- Create a complete ROS 2 package with multiple nodes
- Implement publish-subscribe communication patterns
- Use parameters to configure node behavior
- Create and use launch files to manage complex systems
- Debug and troubleshoot common ROS 2 communication issues

## Exercise Overview

In this exercise, you'll build upon the concepts learned in Module 2 to create a more complex robotic system. You'll implement a navigation system that includes sensor processing, path planning, and obstacle avoidance. This exercise will demonstrate how ROS 2's communication patterns enable the development of physical AI systems.

## Part A: Package Creation and Setup (30 minutes)

### Task 1: Create a New Package

1. **Create a new ROS 2 package** called `navigation_system`:
   ```bash
   cd ~/physical_ai_ws/src
   ros2 pkg create --cpp --dependencies rclcpp std_msgs sensor_msgs geometry_msgs navigation_system
   ```

2. **Verify the package structure**:
   ```bash
   ls -la ~/physical_ai_ws/src/navigation_system
   ```

3. **Update the package.xml** with proper dependencies:
   ```xml
   <depend>rclcpp</depend>
   <depend>std_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>geometry_msgs</depend>
   <depend>nav_msgs</depend>
   ```

## Part B: Implement the Navigation System (90 minutes)

### Task 2: Create the Sensor Processing Node

Create a node that processes sensor data to detect obstacles and free space:

**File: `src/sensor_processor.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

class SensorProcessor : public rclcpp::Node
{
public:
    SensorProcessor() : Node("sensor_processor")
    {
        // Create subscription to laser scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SensorProcessor::scan_callback, this, std::placeholders::_1));
        
        // Create publisher for processed sensor data
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Declare parameters
        this->declare_parameter<double>("safety_distance", 1.0);
        this->declare_parameter<double>("linear_speed", 0.3);
        this->declare_parameter<double>("angular_speed", 0.5);
        
        RCLCPP_INFO(this->get_logger(), "Sensor processor initialized");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process scan to detect obstacles
        double min_range = std::numeric_limits<double>::max();
        for (const auto& range : msg->ranges) {
            if (range > msg->range_min && range < msg->range_max && range < min_range) {
                min_range = range;
            }
        }
        
        // Publish obstacle detection
        auto obstacle_msg = std_msgs::msg::Bool();
        obstacle_msg.data = (min_range < this->get_parameter("safety_distance").as_double());
        obstacle_pub_->publish(obstacle_msg);
        
        // Generate simple velocity commands based on sensor data
        auto cmd_msg = geometry_msgs::msg::Twist();
        if (obstacle_msg.data) {
            // Stop if obstacle detected
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
        } else {
            // Move forward if no obstacle
            cmd_msg.linear.x = this->get_parameter("linear_speed").as_double();
            cmd_msg.angular.z = 0.0;
        }
        
        cmd_pub_->publish(cmd_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorProcessor>());
    rclcpp::shutdown();
    return 0;
}
```

### Task 3: Create the Path Planning Node

Create a node that implements simple path planning based on goals:

**File: `src/path_planner.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        // Create publishers and subscribers
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PathPlanner::goal_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PathPlanner::odom_callback, this, std::placeholders::_1));
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Declare parameters
        this->declare_parameter<double>("linear_speed", 0.5);
        this->declare_parameter<double>("angular_speed", 0.5);
        this->declare_parameter<double>("goal_tolerance", 0.5);
        
        RCLCPP_INFO(this->get_logger(), "Path planner initialized");
    }

private:
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = msg->pose;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", 
                   goal_pose_.position.x, goal_pose_.position.y);
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        
        if (goal_received_) {
            // Calculate control commands to reach goal
            auto cmd_msg = geometry_msgs::msg::Twist();
            
            // Calculate distance to goal
            double dx = goal_pose_.position.x - current_pose_.position.x;
            double dy = goal_pose_.position.y - current_pose_.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance > this->get_parameter("goal_tolerance").as_double()) {
                // Calculate angle to goal
                double goal_yaw = std::atan2(dy, dx);
                
                // Get current yaw from quaternion
                tf2::Quaternion q(
                    current_pose_.orientation.x,
                    current_pose_.orientation.y,
                    current_pose_.orientation.z,
                    current_pose_.orientation.w
                );
                double current_yaw;
                tf2::Matrix3x3 m(q);
                m.getRPY(roll_, pitch_, current_yaw);
                
                // Calculate angular error
                double angle_error = goal_yaw - current_yaw;
                
                // Normalize angle to [-pi, pi]
                while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
                while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
                
                // Set velocity commands
                if (std::abs(angle_error) > 0.1) {
                    // Turn toward goal
                    cmd_msg.angular.z = std::min(
                        std::max(angle_error * 0.5, -this->get_parameter("angular_speed").as_double()),
                        this->get_parameter("angular_speed").as_double()
                    );
                } else {
                    // Move forward toward goal
                    cmd_msg.linear.x = std::min(
                        this->get_parameter("linear_speed").as_double(),
                        distance * 0.5  // Slow down as we approach the goal
                    );
                }
            } else {
                // Goal reached
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                goal_received_ = false;
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
            }
            
            cmd_pub_->publish(cmd_msg);
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    geometry_msgs::msg::Pose goal_pose_;
    geometry_msgs::msg::Pose current_pose_;
    bool goal_received_ = false;
    double roll_, pitch_, yaw_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
```

### Task 4: Update CMakeLists.txt

Add the new executables to your CMakeLists.txt:

```cmake
# Add the sensor processor executable
add_executable(sensor_processor src/sensor_processor.cpp)
ament_target_dependencies(sensor_processor
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Add the path planner executable
add_executable(path_planner src/path_planner.cpp)
ament_target_dependencies(path_planner
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  sensor_processor
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)
```

## Part C: Create Launch Files (30 minutes)

### Task 5: Create a Launch File

Create a launch file to run your navigation system:

**File: `launch/navigation_system.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensor processor node
        Node(
            package='navigation_system',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[
                {'safety_distance': 1.0},
                {'linear_speed': 0.3},
                {'angular_speed': 0.5}
            ],
            output='screen'
        ),
        
        # Path planner node
        Node(
            package='navigation_system',
            executable='path_planner',
            name='path_planner',
            parameters=[
                {'linear_speed': 0.5},
                {'angular_speed': 0.5},
                {'goal_tolerance': 0.5}
            ],
            output='screen'
        )
    ])
```

## Part D: Build and Test the System (45 minutes)

### Task 6: Build the Package

```bash
cd ~/physical_ai_ws
colcon build --packages-select navigation_system
source install/setup.bash
```

### Task 7: Test the System

1. **Run the launch file:**
   ```bash
   ros2 launch navigation_system navigation_system.launch.py
   ```

2. **In another terminal, publish a test laser scan:**
   ```bash
   # Publish a simple scan with no obstacles (all ranges at 3.0 meters)
   ros2 topic pub /scan sensor_msgs/msg/LaserScan "header:
     stamp:
       sec: 0
       nanosec: 0
     frame_id: 'laser_frame'
   angle_min: -1.57
   angle_max: 1.57
   angle_increment: 0.01745
   time_increment: 0.0
   scan_time: 0.1
   range_min: 0.1
   range_max: 10.0
   ranges: [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]" -1
   ```

3. **Monitor the output:**
   ```bash
   # Check if obstacle detection is working
   ros2 topic echo /obstacle_detected
   
   # Check velocity commands
   ros2 topic echo /cmd_vel
   ```

### Task 8: Test with Obstacle

1. **Publish a scan with an obstacle:**
   ```bash
   ros2 topic pub /scan sensor_msgs/msg/LaserScan "header:
     stamp:
       sec: 0
       nanosec: 0
     frame_id: 'laser_frame'
   angle_min: -1.57
   angle_max: 1.57
   angle_increment: 0.01745
   time_increment: 0.0
   scan_time: 0.1
   range_min: 0.1
   range_max: 10.0
   ranges: [0.5, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]" -1
   ```

2. **Observe how the system responds to the obstacle.**

## Part E: Parameter Tuning and Analysis (30 minutes)

### Task 9: Experiment with Parameters

1. **Modify the safety distance parameter:**
   ```bash
   ros2 run navigation_system sensor_processor --ros-args -p safety_distance:=0.8
   ```

2. **Modify the linear speed parameter:**
   ```bash
   ros2 run navigation_system sensor_processor --ros-args -p linear_speed:=0.2
   ```

3. **Document how changing parameters affects system behavior.**

## Part F: Extension Challenge (Optional, 45 minutes)

### Task 10: Enhance the System (Optional)

Enhance your navigation system by implementing one of these features:

1. **Add a simple mapping component** that stores obstacles detected by the sensor processor
2. **Implement a more sophisticated obstacle avoidance algorithm** that plans a path around obstacles
3. **Add a safety node** that monitors all velocity commands and ensures they stay within safe limits

## Submission Requirements

Submit the following for this exercise:

1. **Complete source code** for all nodes implemented
2. **Updated CMakeLists.txt and package.xml** files
3. **Launch file** for the navigation system
4. **Documentation** of parameter tuning results
5. **Brief reflection** (1-2 paragraphs) on how ROS 2's communication patterns enable physical AI systems

## Evaluation Criteria

- **Package Structure (20%)**: Properly organized ROS 2 package with correct dependencies
- **Node Implementation (40%)**: Working sensor processor and path planner nodes with appropriate communication
- **Launch Configuration (20%)**: Properly configured launch file that runs the system
- **Testing and Analysis (20%)**: Evidence of testing and parameter tuning with analysis of results

## Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Navigation System Resources: [Course Resources Section]