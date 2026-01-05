# Module 1 Exercise: Physical AI Fundamentals

## Learning Objectives

After completing this exercise, you will be able to:
- Differentiate between digital AI and physical AI systems
- Apply embodied intelligence principles to simple robotic scenarios
- Set up and validate a basic ROS 2 development environment
- Create a simple ROS 2 node that demonstrates basic concepts

## Exercise Overview

This exercise combines theoretical understanding with practical implementation. You'll first analyze scenarios to distinguish between digital and physical AI, then implement a simple ROS 2 node that demonstrates the principles of embodied intelligence.

## Part A: Conceptual Analysis (30 minutes)

### Task 1: Digital vs. Physical AI Classification

For each of the following scenarios, classify whether it represents digital AI, physical AI, or both, and justify your answer:

1. **A chess-playing AI system**
   - Classification: ________________
   - Justification: 

2. **An autonomous vehicle navigating city streets**
   - Classification: ________________
   - Justification: 

3. **A recommendation algorithm for e-commerce**
   - Classification: ________________
   - Justification: 

4. **A robot vacuum cleaner**
   - Classification: ________________
   - Justification: 

5. **A language translation model**
   - Classification: ________________
   - Justification: 

6. **A humanoid robot serving drinks at a party**
   - Classification: ________________
   - Justification: 

### Task 2: Embodied Intelligence Principles

For each principle of embodied intelligence, provide a specific example of how it applies to a humanoid robot:

1. **Embodiment Principle**
   - Example: 

2. **Environmental Interaction Principle**
   - Example: 

3. **Situatedness Principle**
   - Example: 

4. **Emergence Principle**
   - Example: 

## Part B: Environment Setup and Validation (45 minutes)

### Task 3: Validate Your ROS 2 Environment

1. **Verify ROS 2 installation:**
   ```bash
   printenv | grep -i ros
   ```

2. **Check ROS 2 distribution:**
   ```bash
   echo $ROS_DISTRO
   ```

3. **Test basic ROS 2 functionality:**
   ```bash
   # In one terminal, run:
   ros2 run demo_nodes_cpp talker
   
   # In another terminal, run:
   ros2 run demo_nodes_cpp listener
   ```

4. **Document your results:**
   - What did you observe when running the talker/listener example?
   - What does this demonstrate about ROS 2 communication?

### Task 4: Verify Simulation Environment

1. **Check Gazebo installation:**
   ```bash
   gz --version
   ```

2. **Launch basic Gazebo simulation:**
   ```bash
   gz sim
   ```

3. **Document your observations:**
   - What do you see in the Gazebo interface?
   - How does this differ from a traditional video game or application?

## Part C: Simple ROS 2 Node Implementation (60 minutes)

### Task 5: Create a Simple Physical AI Node

In this task, you'll create a simple ROS 2 node that demonstrates basic principles of physical AI. This node will simulate a simple sensorimotor loop where a robot responds to proximity information.

1. **Create a new ROS 2 package:**
   ```bash
   cd ~/physical_ai_ws/src
   ros2 pkg create --cpp --dependencies rclcpp std_msgs geometry_msgs sensor_msgs physical_ai_exercise_1
   ```

2. **Navigate to the package directory:**
   ```bash
   cd ~/physical_ai_ws/src/physical_ai_exercise_1
   ```

3. **Create the source file for your node:**
   ```bash
   touch src/proximity_sensor_node.cpp
   ```

4. **Implement the node in `src/proximity_sensor_node.cpp`:**
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "sensor_msgs/msg/laser_scan.hpp"
   #include "geometry_msgs/msg/twist.hpp"

   class ProximitySensorNode : public rclcpp::Node
   {
   public:
       ProximitySensorNode() : Node("proximity_sensor_node")
       {
           // Create a publisher for velocity commands
           vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
           
           // Create a subscription for laser scan data
           scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
               "/scan", 10,
               std::bind(&ProximitySensorNode::scan_callback, this, std::placeholders::_1));
               
           RCLCPP_INFO(this->get_logger(), "Proximity Sensor Node Initialized");
       }

   private:
       void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
       {
           // Find the minimum distance in the scan
           float min_distance = std::numeric_limits<float>::max();
           for (auto range : msg->ranges) {
               if (range < min_distance && range > msg->range_min) {
                   min_distance = range;
               }
           }
           
           // Create a velocity command based on proximity
           auto vel_msg = geometry_msgs::msg::Twist();
           
           if (min_distance < 1.0) {  // If obstacle is closer than 1 meter
               // Turn to avoid obstacle
               vel_msg.linear.x = 0.0;
               vel_msg.angular.z = 0.5;  // Turn right
               RCLCPP_WARN(this->get_logger(), "Obstacle detected! Turning right.");
           } else {
               // Move forward
               vel_msg.linear.x = 0.5;   // Move forward at 0.5 m/s
               vel_msg.angular.z = 0.0;
               RCLCPP_INFO(this->get_logger(), "Clear path, moving forward.");
           }
           
           // Publish the velocity command
           vel_publisher_->publish(vel_msg);
       }
       
       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
       rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
   };

   int main(int argc, char * argv[])
   {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<ProximitySensorNode>());
       rclcpp::shutdown();
       return 0;
   }
   ```

5. **Update the CMakeLists.txt file:**
   Add the following before the `ament_package()` line:
   ```cmake
   find_package(geometry_msgs REQUIRED)
   find_package(sensor_msgs REQUIRED)

   add_executable(proximity_sensor_node src/proximity_sensor_node.cpp)
   ament_target_dependencies(proximity_sensor_node 
       rclcpp 
       std_msgs 
       geometry_msgs 
       sensor_msgs)

   install(TARGETS
     proximity_sensor_node
     DESTINATION lib/${PROJECT_NAME})
   ```

6. **Build the package:**
   ```bash
   cd ~/physical_ai_ws
   colcon build --packages-select physical_ai_exercise_1
   source install/setup.bash
   ```

### Task 6: Test Your Node (30 minutes)

1. **Run your node:**
   ```bash
   ros2 run physical_ai_exercise_1 proximity_sensor_node
   ```

2. **Note:** Since we don't yet have a simulation with laser scan data, you'll see the node waiting for scan messages. This is expected for now.

3. **In a new terminal, check that your node is running:**
   ```bash
   ros2 node list
   ros2 topic list
   ```

4. **Document your observations:**
   - What nodes and topics do you see?
   - What would happen if you had a simulated robot publishing laser scan data to the `/scan` topic?

## Part D: Reflection and Analysis (30 minutes)

### Task 7: Connect Implementation to Theory

1. **How does your proximity sensor node demonstrate the principles of embodied intelligence?**
   - Which principle(s) does it exemplify?
   - How does the sensorimotor loop work in your implementation?

2. **What are the advantages of this approach compared to a purely digital AI system?**
   - How does real-time interaction with the environment enhance the system's capabilities?
   - What would be difficult to implement with a purely digital approach?

3. **What challenges might arise when deploying this type of system in the real world?**
   - Consider factors like sensor noise, real-time constraints, and safety.

## Part E: Extension Challenge (Optional, 30 minutes)

### Task 8: Enhance the Node (Optional)

Modify your node to implement more sophisticated behavior:

1. **Add hysteresis** to prevent oscillation when an obstacle is near the threshold
2. **Implement proportional control** where the turning rate is proportional to how close the obstacle is
3. **Add random movement** when completely stuck to prevent infinite loops

## Submission Requirements

Submit the following for this exercise:

1. **Completed answers** to all conceptual analysis questions
2. **Screenshots** of your environment validation results
3. **Your complete source code** for the proximity sensor node
4. **A brief reflection** (1-2 paragraphs) on how this exercise helped you understand the difference between digital and physical AI

## Evaluation Criteria

- **Conceptual Understanding (40%)**: Correct classification and explanation of digital vs. physical AI scenarios
- **Implementation (40%)**: Working ROS 2 node that demonstrates sensorimotor principles
- **Analysis (20%)**: Thoughtful reflection on the connection between theory and implementation

## Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo Simulation: https://gazebosim.org/
- Physical AI Resources: [Course Resources Section]