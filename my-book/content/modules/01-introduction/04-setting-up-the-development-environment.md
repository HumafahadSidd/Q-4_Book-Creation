# Chapter 1.4: Setting Up the Development Environment

## Learning Objectives

By the end of this chapter, students will be able to:
- Install and configure the ROS 2 development environment
- Set up simulation environments for robotics development
- Configure essential tools for physical AI development
- Validate the installation and configuration
- Troubleshoot common setup issues

## Introduction

Setting up a proper development environment is the crucial first step in working with physical AI and humanoid robotics. The tools and frameworks you'll use throughout this course require careful installation and configuration to ensure smooth operation. This chapter provides detailed instructions for setting up your development environment, including the Robot Operating System (ROS 2), simulation platforms, and essential development tools.

The development environment we'll establish serves as the foundation for all subsequent work in this course. It includes tools for simulation, development, testing, and deployment of physical AI systems. Taking the time to properly set up this environment will save you significant time and frustration as you progress through the course.

## System Requirements

Before beginning the installation, ensure your system meets the following requirements:

### Minimum Hardware Requirements
- **Processor**: Multi-core processor (Intel i5 or equivalent)
- **Memory**: 8 GB RAM (16 GB recommended)
- **Storage**: 50 GB free disk space
- **Graphics**: GPU with OpenGL 3.3 support (for simulation)
- **Network**: Internet connection for package downloads

### Recommended Hardware Requirements
- **Processor**: Intel i7 or AMD Ryzen 7 (or better)
- **Memory**: 16 GB RAM (32 GB for intensive simulation)
- **Storage**: 100 GB SSD (for faster compilation and simulation)
- **Graphics**: Dedicated GPU with CUDA support (for NVIDIA Isaac)
- **Network**: High-speed internet connection

### Supported Operating Systems
- **Ubuntu 22.04 LTS** (Primary support)
- **Windows 11** (with WSL2 or native installation)
- **macOS** (with Docker or native installation)

## Installing ROS 2 (Humble Hawksbill)

ROS 2 (Robot Operating System 2) is the middleware framework that enables communication between different components of your robotic system. We'll be using the Humble Hawksbill distribution, which is an LTS (Long Term Support) release with extended support.

### On Ubuntu 22.04

1. **Set up locale**
   ```bash
   locale  # check for UTF-8
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US.UTF-8
   ```

2. **Set up sources**
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   ```

3. **Add ROS 2 apt repository**
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **Install ROS 2 packages**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. **Install colcon build tools**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

6. **Install ROS development tools**
   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

7. **Initialize rosdep**
   ```bash
   sudo rosdep init
   rosdep update
   ```

8. **Environment setup**
   Add the following line to your `~/.bashrc` file:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### On Windows 11 with WSL2

1. **Install WSL2 with Ubuntu 22.04**
   - Open PowerShell as Administrator and run:
     ```powershell
     wsl --install -d Ubuntu-22.04
     ```

2. **Follow Ubuntu installation instructions** inside the WSL terminal

3. **Install VcXsrv or WSLg** for GUI applications
   - For VcXsrv: Install and run with proper settings for X11 forwarding
   - For WSLg: Available in Windows 11 with WSL kernel update

4. **Configure display forwarding**
   ```bash
   export DISPLAY=:0
   # Or for network-based forwarding:
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0
   ```

### On Windows 11 (Native Installation)

1. **Install Visual Studio Community 2022**
   - Include C++ development tools

2. **Install Python 3.8 or later**
   - Add to PATH during installation

3. **Download ROS 2 Humble installer**
   - From the official ROS 2 website
   - Run the Windows installer

4. **Configure environment variables**
   - Add ROS 2 installation path to PATH
   - Set ROS_DOMAIN_ID if needed

## Installing Gazebo Simulation Environment

Gazebo is a 3D simulation environment that allows you to test your robotic systems in realistic virtual worlds before deploying them on real hardware.

### On Ubuntu 22.04

1. **Install Gazebo Garden** (recommended version for ROS 2 Humble)
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository ppa:openrobotics/garden
   sudo apt update
   sudo apt install gz-garden
   ```

2. **Install ROS 2 Gazebo bridge**
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

3. **Test the installation**
   ```bash
   gz sim
   ```

### On Windows with WSL2

1. **Install Gazebo Garden** following Ubuntu instructions inside WSL2

2. **Ensure X11 forwarding is configured** for GUI display

## Installing NVIDIA Isaac Sim (Optional Advanced Setup)

NVIDIA Isaac Sim provides advanced simulation capabilities for perception, navigation, and manipulation tasks using NVIDIA's GPU-accelerated technologies.

### Prerequisites
- NVIDIA GPU with CUDA support (RTX series recommended)
- CUDA 11.8 or later installed
- NVIDIA drivers (520 or later)

### Installation Steps

1. **Install Docker** (if not already installed)
   ```bash
   sudo apt install docker.io
   sudo usermod -aG docker $USER
   # Log out and back in for changes to take effect
   ```

2. **Pull Isaac Sim Docker image**
   ```bash
   docker pull nvcr.io/nvidia/isaac-sim:latest
   ```

3. **Run Isaac Sim container**
   ```bash
   docker run --gpus all -it --rm --network=host \
     --env "ACCEPT_EULA=Y" --env "NVIDIA_VISIBLE_DEVICES=all" \
     --volume $(pwd):/workspace/isaac-sim/shared \
     --volume ~/.nvidia-isaac-sim/cache/kit:/isaac-sim/kit/cache/global \
     --volume ~/.nvidia-isaac-sim/cache/ov:/root/.cache/ov \
     --volume ~/.nvidia-isaac-sim/cache/kit:/root/.local/share/ov/pkg/kit-v1.37.3/cache \
     --volume ~/.nvidia-isaac-sim/logs:/root/.nvidia-isaac-sim/logs \
     --volume ~/.nvidia-isaac-sim/config:/root/.nvidia-isaac-sim/config \
     nvcr.io/nvidia/isaac-sim:latest
   ```

## Installing Additional Development Tools

### Git and Version Control
```bash
sudo apt install git git-lfs
```

### Python Development Tools
```bash
sudo apt install python3-pip python3-dev
pip3 install --user -U argcomplete
```

### Code Editor
While you can use any editor, we recommend:
- **VS Code** with ROS extension
- **PyCharm** for Python development
- **CLion** for C++ development

### Install VS Code with ROS extension
```bash
sudo snap install code --classic
code --install-extension ms-iot.vscode-ros
```

## Creating a Workspace

Now that we have the necessary tools installed, let's create a workspace for our ROS 2 development:

1. **Create the workspace directory**
   ```bash
   mkdir -p ~/physical_ai_ws/src
   cd ~/physical_ai_ws
   ```

2. **Source ROS 2 environment**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the workspace**
   ```bash
   colcon build
   ```

4. **Source the workspace**
   ```bash
   source install/setup.bash
   ```

5. **Add workspace sourcing to bashrc**
   ```bash
   echo "source ~/physical_ai_ws/install/setup.bash" >> ~/.bashrc
   ```

## Validating the Installation

Let's run some basic tests to ensure everything is working correctly:

### Test ROS 2 Installation
1. **Open a new terminal** (to source the updated bashrc)

2. **Check ROS 2 installation**
   ```bash
   printenv | grep -i ros
   ```

3. **Run a simple ROS 2 demo**
   ```bash
   ros2 run demo_nodes_cpp talker
   ```
   (Open another terminal and run `ros2 run demo_nodes_cpp listener` to see the communication)

### Test Gazebo Installation
```bash
gz sim --headless-validation
```

### Test Python Integration
```bash
python3 -c "import rclpy; print('ROS 2 Python client is working')"
```

## Troubleshooting Common Issues

### Issue: "command not found" for ROS 2 commands
**Solution:** Ensure ROS 2 environment is sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Gazebo fails to start with graphics errors
**Solution:** Check graphics drivers and ensure proper X11 forwarding if using WSL2

### Issue: Permission denied when building workspace
**Solution:** Ensure proper ownership of workspace directory:
```bash
sudo chown -R $USER:$USER ~/physical_ai_ws
```

### Issue: Python packages not found
**Solution:** Ensure Python paths are correctly set:
```bash
echo $PYTHONPATH
export PYTHONPATH=$AMENT_PREFIX_PATH/lib/python3.10/site-packages:$PYTHONPATH
```

### Issue: CUDA-related errors with Isaac Sim
**Solution:** Verify CUDA installation and GPU compatibility:
```bash
nvidia-smi
nvcc --version
```

## Setting Up for the Course

Now that your environment is set up, let's create a project structure that will be used throughout the course:

1. **Create course-specific directories**
   ```bash
   cd ~/physical_ai_ws/src
   mkdir -p physical_ai_book/{common,chapter2,chapter3,chapter4,chapter5,chapter6,chapter7,chapter8,chapter9,chapter10}
   ```

2. **Initialize Git repository for your work**
   ```bash
   cd ~/physical_ai_ws
   git init
   git remote add origin <your_repository_url>  # if using remote repo
   ```

3. **Create initial package for Chapter 1**
   ```bash
   cd ~/physical_ai_ws/src
   ros2 pkg create --cpp --description "Chapter 1: Introduction to Physical AI" physical_ai_chapter1
   ```

## Essential Development Practices

### 1. Workspace Organization
- Keep separate packages for different modules/chapters
- Use descriptive package names
- Maintain clear README files for each package

### 2. Version Control
- Commit regularly with descriptive messages
- Use feature branches for experimental work
- Keep a clean commit history

### 3. Testing and Validation
- Test components individually before integration
- Use simulation to validate before real hardware deployment
- Document test results and validation procedures

## Next Steps

With your development environment properly set up, you're now ready to begin implementing physical AI systems. The next module will introduce you to ROS 2 fundamentals, where you'll learn to create your first robotic nodes and understand the communication patterns that form the backbone of robotic systems.

Your environment includes:
- ROS 2 Humble Hawksbill for robotic communication
- Gazebo simulation environment for testing
- Essential development tools for coding and debugging
- A properly structured workspace for course projects

Take time to familiarize yourself with the tools and verify that all components are working correctly before proceeding to the next chapter. In the next module, we'll dive into ROS 2 fundamentals and create our first robotic applications.