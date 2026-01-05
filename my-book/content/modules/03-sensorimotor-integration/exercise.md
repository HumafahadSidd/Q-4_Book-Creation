# Module 3 Exercise: Sensorimotor Integration and Perception-Action Loops

## Learning Objectives

After completing this exercise, you will be able to:
- Implement sensor data processing and filtering techniques
- Design and implement perception-action loops
- Apply feedback control to robotic systems
- Integrate sensors, processing, and control in a complete system
- Evaluate the performance of sensorimotor systems

## Exercise Overview

In this exercise, you'll build a complete sensorimotor system that processes LIDAR data to navigate a robot around obstacles. You'll implement filtering techniques, create a perception-action loop, and apply feedback control to achieve the navigation task.

## Part A: Sensor Data Processing (45 minutes)

### Task 1: Implement a LIDAR Data Filter

Create a ROS 2 node that processes raw LIDAR data to remove noise and outliers:

**File: `~/physical_ai_ws/src/sensorimotor_exercises/src/lidar_filter_node.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
    LidarFilterNode() : Node("lidar_filter_node")
    {
        // Create subscription to raw scan
        raw_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10,
            std::bind(&LidarFilterNode::rawScanCallback, this, std::placeholders::_1));
        
        // Create publisher for filtered scan
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
        
        RCLCPP_INFO(this->get_logger(), "LIDAR Filter Node initialized");
    }

private:
    void rawScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create filtered message
        auto filtered_msg = *msg;  // Copy header and metadata
        
        // Apply filtering to ranges
        filtered_msg.ranges = filterLidarData(msg->ranges, msg->range_min, msg->range_max);
        
        // Publish filtered data
        filtered_scan_pub_->publish(filtered_msg);
    }
    
    std::vector<float> filterLidarData(const std::vector<float>& raw_ranges, 
                                      float range_min, float range_max)
    {
        std::vector<float> filtered_ranges = raw_ranges;
        
        // TODO: Implement the following filtering techniques:
        // 1. Remove readings outside valid range (range_min to range_max)
        // 2. Apply a median filter to remove outliers
        // 3. Apply a simple moving average to smooth data
        
        // Step 1: Replace out-of-range values with max range
        for (auto& range : filtered_ranges) {
            if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range)) {
                range = range_max;  // Set to max range for invalid readings
            }
        }
        
        // Step 2: Apply median filter to remove outliers
        filtered_ranges = applyMedianFilter(filtered_ranges, 3);  // Window size of 3
        
        // Step 3: Apply moving average to smooth data
        filtered_ranges = applyMovingAverage(filtered_ranges, 3);  // Window size of 3
        
        return filtered_ranges;
    }
    
    std::vector<float> applyMedianFilter(const std::vector<float>& input, int window_size)
    {
        std::vector<float> output = input;
        int half_window = window_size / 2;
        
        for (size_t i = half_window; i < input.size() - half_window; ++i) {
            std::vector<float> window_values;
            
            // Collect values in the window
            for (int j = -half_window; j <= half_window; ++j) {
                window_values.push_back(input[i + j]);
            }
            
            // Sort to find median
            std::sort(window_values.begin(), window_values.end());
            
            // Replace with median value
            output[i] = window_values[window_values.size() / 2];
        }
        
        return output;
    }
    
    std::vector<float> applyMovingAverage(const std::vector<float>& input, int window_size)
    {
        std::vector<float> output = input;
        int half_window = window_size / 2;
        
        for (size_t i = half_window; i < input.size() - half_window; ++i) {
            float sum = 0.0;
            
            // Sum values in the window
            for (int j = -half_window; j <= half_window; ++j) {
                sum += input[i + j];
            }
            
            // Calculate average
            output[i] = sum / window_size;
        }
        
        return output;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilterNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Task 2: Test Your Filter

1. **Build your package:**
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select sensorimotor_exercises
   source install/setup.bash
   ```

2. **Run your filter node:**
   ```bash
   ros2 run sensorimotor_exercises lidar_filter_node
   ```

3. **Test with sample data:**
   ```bash
   # In another terminal, publish test scan data
   ros2 topic pub /scan_raw sensor_msgs/msg/LaserScan "{
     header: {frame_id: 'laser_frame'},
     angle_min: -1.57, angle_max: 1.57, angle_increment: 0.01745,
     time_increment: 0.0, scan_time: 0.1, range_min: 0.1, range_max: 10.0,
     ranges: [0.5, 0.6, 10.0, 0.8, 0.7, 0.0, 1.2, 1.1, 1.0, 1.3]
   }" -1
   ```

4. **Monitor the output:**
   ```bash
   ros2 topic echo /scan_filtered
   ```

## Part B: Perception-Action Loop (60 minutes)

### Task 3: Implement an Obstacle Avoidance System

Create a perception-action loop that uses filtered LIDAR data to avoid obstacles:

**File: `~/physical_ai_ws/src/sensorimotor_exercises/src/obstacle_avoidance_node.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node")
    {
        // Create subscription to filtered scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_filtered", 10,
            std::bind(&ObstacleAvoidanceNode::scanCallback, this, std::placeholders::_1));
        
        // Create publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process scan data to detect obstacles
        auto [min_distance, min_angle] = findClosestObstacle(*msg);
        
        // Create velocity command based on obstacle detection
        auto cmd = geometry_msgs::msg::Twist();
        
        if (min_distance < safety_distance_) {
            // Obstacle detected - turn away
            cmd.linear.x = 0.0;  // Stop forward motion
            
            // Turn in the direction away from the obstacle
            if (min_angle < 0) {
                // Obstacle on the left - turn right
                cmd.angular.z = -turn_speed_;
            } else {
                // Obstacle on the right - turn left
                cmd.angular.z = turn_speed_;
            }
            
            RCLCPP_WARN(this->get_logger(), 
                       "Obstacle detected at %.2f m, angle %.2f rad, turning", 
                       min_distance, min_angle);
        } else {
            // Clear path - move forward
            cmd.linear.x = forward_speed_;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Clear path, moving forward");
        }
        
        // Publish command
        cmd_pub_->publish(cmd);
    }
    
    std::pair<float, float> findClosestObstacle(const sensor_msgs::msg::LaserScan& scan)
    {
        float min_distance = std::numeric_limits<float>::max();
        float min_angle = 0.0;
        int min_index = -1;
        
        // Look for closest obstacle in front (front 180 degrees)
        size_t start_idx = scan.ranges.size() / 4;  // 45 degrees from center
        size_t end_idx = 3 * scan.ranges.size() / 4;  // 45 degrees from center
        
        for (size_t i = start_idx; i < end_idx; ++i) {
            if (scan.ranges[i] > scan.range_min && 
                scan.ranges[i] < scan.range_max && 
                scan.ranges[i] < min_distance) {
                
                min_distance = scan.ranges[i];
                min_index = i;
            }
        }
        
        if (min_index != -1) {
            min_angle = scan.angle_min + min_index * scan.angle_increment;
        }
        
        return std::make_pair(min_distance, min_angle);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    // Parameters
    float safety_distance_ = 1.0;  // meters
    float forward_speed_ = 0.3;   // m/s
    float turn_speed_ = 0.5;      // rad/s
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
```

### Task 4: Create a Combined Node

Create a single node that combines filtering and obstacle avoidance:

**File: `~/physical_ai_ws/src/sensorimotor_exercises/src/sensorimotor_controller.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SensorimotorController : public rclcpp::Node
{
public:
    SensorimotorController() : Node("sensorimotor_controller")
    {
        // Create subscription to raw scan
        raw_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10,
            std::bind(&SensorimotorController::rawScanCallback, this, std::placeholders::_1));
        
        // Create publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Sensorimotor Controller initialized");
    }

private:
    void rawScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Apply filtering to raw scan
        auto filtered_ranges = filterLidarData(msg->ranges, msg->range_min, msg->range_max);
        
        // Process filtered data to detect obstacles
        float min_distance;
        float min_angle;
        std::tie(min_distance, min_angle) = findClosestObstacle(filtered_ranges, *msg);
        
        // Create velocity command based on obstacle detection
        auto cmd = geometry_msgs::msg::Twist();
        
        if (min_distance < safety_distance_) {
            // Obstacle detected - turn away
            cmd.linear.x = 0.0;  // Stop forward motion
            
            // Turn in the direction away from the obstacle
            if (min_angle < 0) {
                // Obstacle on the left - turn right
                cmd.angular.z = -turn_speed_;
            } else {
                // Obstacle on the right - turn left
                cmd.angular.z = turn_speed_;
            }
            
            RCLCPP_WARN(this->get_logger(), 
                       "Obstacle detected at %.2f m, angle %.2f rad, turning", 
                       min_distance, min_angle);
        } else {
            // Clear path - move forward
            cmd.linear.x = forward_speed_;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Clear path, moving forward");
        }
        
        // Publish command
        cmd_pub_->publish(cmd);
    }
    
    std::vector<float> filterLidarData(const std::vector<float>& raw_ranges, 
                                      float range_min, float range_max)
    {
        std::vector<float> filtered_ranges = raw_ranges;
        
        // Remove out-of-range values
        for (auto& range : filtered_ranges) {
            if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range)) {
                range = range_max;
            }
        }
        
        // Apply median filter
        filtered_ranges = applyMedianFilter(filtered_ranges, 3);
        
        // Apply moving average
        filtered_ranges = applyMovingAverage(filtered_ranges, 3);
        
        return filtered_ranges;
    }
    
    std::vector<float> applyMedianFilter(const std::vector<float>& input, int window_size)
    {
        std::vector<float> output = input;
        int half_window = window_size / 2;
        
        for (size_t i = half_window; i < input.size() - half_window; ++i) {
            std::vector<float> window_values;
            
            for (int j = -half_window; j <= half_window; ++j) {
                window_values.push_back(input[i + j]);
            }
            
            std::sort(window_values.begin(), window_values.end());
            output[i] = window_values[window_values.size() / 2];
        }
        
        return output;
    }
    
    std::vector<float> applyMovingAverage(const std::vector<float>& input, int window_size)
    {
        std::vector<float> output = input;
        int half_window = window_size / 2;
        
        for (size_t i = half_window; i < input.size() - half_window; ++i) {
            float sum = 0.0;
            
            for (int j = -half_window; j <= half_window; ++j) {
                sum += input[i + j];
            }
            
            output[i] = sum / window_size;
        }
        
        return output;
    }
    
    std::pair<float, float> findClosestObstacle(const std::vector<float>& ranges, 
                                               const sensor_msgs::msg::LaserScan& scan)
    {
        float min_distance = std::numeric_limits<float>::max();
        float min_angle = 0.0;
        int min_index = -1;
        
        // Look for closest obstacle in front (front 180 degrees)
        size_t start_idx = ranges.size() / 4;
        size_t end_idx = 3 * ranges.size() / 4;
        
        for (size_t i = start_idx; i < end_idx; ++i) {
            if (ranges[i] > scan.range_min && 
                ranges[i] < scan.range_max && 
                ranges[i] < min_distance) {
                
                min_distance = ranges[i];
                min_index = i;
            }
        }
        
        if (min_index != -1) {
            min_angle = scan.angle_min + min_index * scan.angle_increment;
        }
        
        return std::make_pair(min_distance, min_angle);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    // Parameters
    float safety_distance_ = 1.0;
    float forward_speed_ = 0.3;
    float turn_speed_ = 0.5;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorimotorController>());
    rclcpp::shutdown();
    return 0;
}
```

## Part C: Feedback Control Implementation (45 minutes)

### Task 5: Implement PID Control for Navigation

Enhance your obstacle avoidance system with PID control for smoother navigation:

**File: `~/physical_ai_ws/src/sensorimotor_exercises/src/pid_navigation_node.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double output_min, double output_max)
        : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max),
          prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double process_variable, double dt)
    {
        if (dt <= 0.0) return 0.0;

        double error = setpoint - process_variable;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term
        integral_ += error * dt;
        // Anti-windup
        integral_ = std::max(std::min(integral_, integral_limit_), -integral_limit_);
        double i_term = ki_ * integral_;

        // Derivative term
        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;

        // Store values for next iteration
        prev_error_ = error;

        // Calculate output
        double output = p_term + i_term + d_term;

        // Apply output limits
        output = std::max(std::min(output, output_max_), output_min_);

        return output;
    }

    void reset()
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double prev_error_, integral_;
    double integral_limit_ = 100.0;
};

class PIDNavigationNode : public rclcpp::Node
{
public:
    PIDNavigationNode() : Node("pid_navigation_node")
    {
        // Create subscription to raw scan
        raw_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10,
            std::bind(&PIDNavigationNode::rawScanCallback, this, std::placeholders::_1));
        
        // Create publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Initialize PID controllers
        linear_pid_ = std::make_unique<PIDController>(1.0, 0.1, 0.05, -0.5, 0.5);   // Linear velocity
        angular_pid_ = std::make_unique<PIDController>(2.0, 0.1, 0.1, -1.0, 1.0);  // Angular velocity
        
        last_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "PID Navigation Node initialized");
    }

private:
    void rawScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).nanoseconds() / 1e9;  // Convert to seconds
        last_time_ = current_time;
        
        // Apply filtering to raw scan
        auto filtered_ranges = filterLidarData(msg->ranges, msg->range_min, msg->range_max);
        
        // Extract features from scan
        auto [front_distance, left_distance, right_distance] = extractFeatures(filtered_ranges, *msg);
        
        // Set targets
        double target_linear = forward_speed_;
        double target_angular = 0.0;
        
        // Adjust targets based on obstacles
        if (front_distance < safety_distance_) {
            // Slow down when obstacle is close
            target_linear = 0.0;
            
            // Turn away from the closer side
            if (left_distance < right_distance) {
                target_angular = -turn_speed_;  // Turn right
            } else {
                target_angular = turn_speed_;   // Turn left
            }
        } else if (front_distance < 2.0) {
            // Slow down as we approach obstacles
            target_linear = forward_speed_ * (front_distance / 2.0);
        }
        
        // Use PID controllers to generate commands
        double linear_cmd = linear_pid_->compute(target_linear, 0.0, dt);  // Current velocity is approx 0
        double angular_cmd = angular_pid_->compute(target_angular, 0.0, dt);  // Current angular vel is approx 0
        
        // Create and publish command
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = linear_cmd;
        cmd.angular.z = angular_cmd;
        
        cmd_pub_->publish(cmd);
    }
    
    std::vector<float> filterLidarData(const std::vector<float>& raw_ranges, 
                                      float range_min, float range_max)
    {
        std::vector<float> filtered_ranges = raw_ranges;
        
        // Remove out-of-range values
        for (auto& range : filtered_ranges) {
            if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range)) {
                range = range_max;
            }
        }
        
        // Apply median filter
        filtered_ranges = applyMedianFilter(filtered_ranges, 3);
        
        // Apply moving average
        filtered_ranges = applyMovingAverage(filtered_ranges, 3);
        
        return filtered_ranges;
    }
    
    std::vector<float> applyMedianFilter(const std::vector<float>& input, int window_size)
    {
        std::vector<float> output = input;
        int half_window = window_size / 2;
        
        for (size_t i = half_window; i < input.size() - half_window; ++i) {
            std::vector<float> window_values;
            
            for (int j = -half_window; j <= half_window; ++j) {
                window_values.push_back(input[i + j]);
            }
            
            std::sort(window_values.begin(), window_values.end());
            output[i] = window_values[window_values.size() / 2];
        }
        
        return output;
    }
    
    std::vector<float> applyMovingAverage(const std::vector<float>& input, int window_size)
    {
        std::vector<float> output = input;
        int half_window = window_size / 2;
        
        for (size_t i = half_window; i < input.size() - half_window; ++i) {
            float sum = 0.0;
            
            for (int j = -half_window; j <= half_window; ++j) {
                sum += input[i + j];
            }
            
            output[i] = sum / window_size;
        }
        
        return output;
    }
    
    std::tuple<float, float, float> extractFeatures(const std::vector<float>& ranges, 
                                                   const sensor_msgs::msg::LaserScan& scan)
    {
        // Define sectors: front, left, right
        size_t total_beams = ranges.size();
        size_t front_start = total_beams / 2 - total_beams / 8;  // Front 45 degrees
        size_t front_end = total_beams / 2 + total_beams / 8;
        
        size_t left_start = total_beams / 4 - total_beams / 16;  // Left 22.5 degrees
        size_t left_end = total_beams / 4 + total_beams / 16;
        
        size_t right_start = 3 * total_beams / 4 - total_beams / 16;  // Right 22.5 degrees
        size_t right_end = 3 * total_beams / 4 + total_beams / 16;
        
        // Find minimum distances in each sector
        float front_min = findMinInRange(ranges, front_start, front_end);
        float left_min = findMinInRange(ranges, left_start, left_end);
        float right_min = findMinInRange(ranges, right_start, right_end);
        
        return std::make_tuple(front_min, left_min, right_min);
    }
    
    float findMinInRange(const std::vector<float>& ranges, size_t start, size_t end)
    {
        float min_val = std::numeric_limits<float>::max();
        
        for (size_t i = start; i < end && i < ranges.size(); ++i) {
            if (ranges[i] > 0.1 && ranges[i] < min_val) {  // Valid range check
                min_val = ranges[i];
            }
        }
        
        return (min_val == std::numeric_limits<float>::max()) ? 10.0 : min_val;  // Return max range if no valid readings
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    std::unique_ptr<PIDController> linear_pid_;
    std::unique_ptr<PIDController> angular_pid_;
    
    rclcpp::Time last_time_;
    
    // Parameters
    float safety_distance_ = 1.0;
    float forward_speed_ = 0.3;
    float turn_speed_ = 0.5;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDNavigationNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Part D: Update CMakeLists.txt (15 minutes)

### Task 6: Add Executables to CMakeLists.txt

Update your package's CMakeLists.txt file to include the new executables:

```cmake
cmake_minimum_required(VERSION 3.8)
project(sensorimotor_exercises)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create the lidar filter node executable
add_executable(lidar_filter_node src/lidar_filter_node.cpp)
ament_target_dependencies(lidar_filter_node
  rclcpp
  sensor_msgs
)

# Create the obstacle avoidance node executable
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Create the sensorimotor controller executable
add_executable(sensorimotor_controller src/sensorimotor_controller.cpp)
ament_target_dependencies(sensorimotor_controller
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Create the PID navigation node executable
add_executable(pid_navigation_node src/pid_navigation_node.cpp)
ament_target_dependencies(pid_navigation_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  lidar_filter_node
  obstacle_avoidance_node
  sensorimotor_controller
  pid_navigation_node
  DESTINATION lib/${PROJECT_NAME}
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

## Part E: Testing and Evaluation (30 minutes)

### Task 7: Test Your Implementation

1. **Build the package:**
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select sensorimotor_exercises
   source install/setup.bash
   ```

2. **Test the LIDAR filter:**
   ```bash
   # Terminal 1: Run the filter node
   ros2 run sensorimotor_exercises lidar_filter_node
   
   # Terminal 2: Publish test data
   ros2 topic pub /scan_raw sensor_msgs/msg/LaserScan "{
     header: {frame_id: 'laser_frame'},
     angle_min: -1.57, angle_max: 1.57, angle_increment: 0.01745,
     time_increment: 0.0, scan_time: 0.1, range_min: 0.1, range_max: 10.0,
     ranges: [0.5, 0.6, 10.0, 0.8, 0.7, 0.0, 1.2, 1.1, 1.0, 1.3]
   }" -1
   
   # Terminal 3: Monitor filtered output
   ros2 topic echo /scan_filtered
   ```

3. **Test the complete sensorimotor system:**
   ```bash
   # Run the PID navigation node
   ros2 run sensorimotor_exercises pid_navigation_node
   
   # Monitor the velocity commands
   ros2 topic echo /cmd_vel
   ```

### Task 8: Performance Evaluation

1. **Analyze the performance of your filtering techniques:**
   - How well does the median filter remove outliers?
   - How does the moving average affect response time?

2. **Evaluate the navigation behavior:**
   - How smoothly does the robot navigate around obstacles?
   - How does the PID control improve the behavior compared to simple reactive control?

3. **Tune the PID parameters:**
   - Adjust the gains (Kp, Ki, Kd) to achieve smoother navigation
   - Document the parameter values that work best for your system

## Part F: Extension Challenge (Optional, 45 minutes)

### Task 9: Implement a More Sophisticated Navigation Strategy (Optional)

Enhance your navigation system with path planning capabilities:

1. **Implement a wall-following behavior** in addition to obstacle avoidance
2. **Add goal-seeking behavior** to navigate toward a specific target
3. **Implement a simple exploration algorithm** to cover an unknown area

## Submission Requirements

Submit the following for this exercise:

1. **Complete source code** for all implemented nodes
2. **Updated CMakeLists.txt** file with proper executable definitions
3. **Documentation** of your PID tuning process and final parameters
4. **Analysis** of filtering performance and navigation behavior
5. **Brief reflection** (1-2 paragraphs) on how sensorimotor integration enables intelligent robotic behavior

## Evaluation Criteria

- **Filtering Implementation (25%)**: Correct implementation of sensor data filtering techniques
- **Perception-Action Loop (30%)**: Proper integration of perception and action components
- **Control System (25%)**: Effective implementation of feedback control
- **Testing and Analysis (20%)**: Thorough testing and performance analysis

## Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Control Systems Resources: [Course Resources Section]
- Sensor Integration Resources: [Course Resources Section]