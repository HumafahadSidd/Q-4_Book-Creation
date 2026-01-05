# Quickstart Guide: Physical AI & Humanoid Robotics Educational Book

## Overview
This guide provides a quick setup for educators and students to begin working with the "Physical AI & Humanoid Robotics" educational materials. The curriculum is structured as a 13-week capstone course covering embodied intelligence, ROS 2, simulation environments, and real-world robotics applications.

## Prerequisites
- Basic programming knowledge (Python preferred)
- Understanding of fundamental AI/ML concepts
- Familiarity with Linux command line (helpful but not required)
- Access to a computer with at least 8GB RAM and 50GB free disk space

## Software Setup

### 1. Install ROS 2 Humble Hawksbill
The curriculum uses ROS 2 Humble Hawksbill (LTS version with 5-year support). Follow the official installation guide for your OS:
- Ubuntu: http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Windows: http://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
- Docker: Available for all platforms as alternative

### 2. Set up Simulation Environment
Choose your primary simulation platform:

**Option A: Gazebo Garden (Recommended for beginners)**
- Install Gazebo following: https://gazebosim.org/docs/garden/install
- Verify installation with: `gz sim --version`

**Option B: NVIDIA Isaac Sim (Advanced users)**
- Requires NVIDIA GPU with RTX or GTX 10xx/20xx/30xx/40xx series
- Download from NVIDIA Developer website
- Follow setup guide in Module 4 content

### 3. Python Environment
Create a virtual environment for course materials:
```bash
python3 -m venv ~/physical-ai-env
source ~/physical-ai-env/bin/activate  # On Windows: ~/physical-ai-env/Scripts/activate
pip install --upgrade pip
```

### 4. Course Materials
Clone or download the course repository:
```bash
git clone [repository-url]
cd [repository-name]
```

## Getting Started with Course Content

### Week 1: Introduction to Physical AI
1. Navigate to the course content directory
2. Read Module 1 content: `content/modules/01-introduction/`
3. Set up your ROS 2 workspace:
   ```bash
   mkdir -p ~/physical_ai_ws/src
   cd ~/physical_ai_ws
   colcon build
   source install/setup.bash
   ```
4. Run the first simulation example:
   ```bash
   ros2 launch [example-package] [first-example].launch.py
   ```

### Week 3: ROS 2 Fundamentals
1. Complete the exercises in `content/modules/03-ros-fundamentals/`
2. Practice creating nodes, topics, and services
3. Use the provided example code in `content/examples/ros2-basics/`

### Week 5: Simulation Environments
1. Follow the Gazebo tutorials in `content/modules/04-simulation/`
2. Create your first robot model using URDF
3. Implement basic navigation in simulation

## Running Course Exercises

Each module contains practical exercises with the following structure:
```
content/exercises/[module-name]/[exercise-name]/
├── README.md              # Exercise instructions
├── requirements.txt       # Python dependencies
├── setup.sh              # Setup script
├── solution/             # Solution (for educators)
└── student/              # Starter code (for students)
```

To run an exercise:
1. Navigate to the exercise directory
2. Install dependencies: `pip install -r requirements.txt`
3. Run setup script: `./setup.sh`
4. Complete the exercise following README instructions

## Capstone Project Setup

The capstone project integrates all concepts from the course. To set up:

1. Review the project requirements in `content/projects/capstone-project/`
2. Set up the complete development environment with all tools
3. Use the project template in `content/projects/capstone-template/`
4. Follow the weekly milestones in the project timeline

## Troubleshooting

### Common Issues
- **ROS 2 workspace not found**: Ensure you've sourced the setup file: `source ~/physical_ai_ws/install/setup.bash`
- **Gazebo not launching**: Check GPU drivers and run `nvidia-smi` (if using NVIDIA GPU)
- **Python package conflicts**: Use the virtual environment: `source ~/physical-ai-env/bin/activate`

### Getting Help
- Check the FAQ in `content/resources/faq.md`
- Use the discussion forums in your learning management system
- Contact your instructor for module-specific questions

## Next Steps
1. Complete the Week 1 readings and setup
2. Join your course's online community
3. Prepare for the first hands-on session
4. Review the 13-week schedule in `content/resources/schedule.md`

## Additional Resources
- Course bibliography: `content/bibliography/`
- Tool documentation links: `content/resources/tool-links.md`
- Troubleshooting guide: `content/resources/troubleshooting.md`