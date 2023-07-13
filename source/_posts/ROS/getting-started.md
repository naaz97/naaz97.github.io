---
title: Getting started with ROS2 
categories: Robotics
---

>This note contains a detail summary of setting up ROS2 and the basic definitions of the ROS2 concepts. Goal of this is to summarise the basic usage of these concepts.

ROS Distro and the version
```bash
ROS_VERSION=2
ROS_DISTRO=humble
OS=Ubuntu 22.04
```
 
## Setting up the environment

### Add the sourcing to shell startup script to access ROS2 commands
Sourcing your ROS 2 installation workspace in the terminal makes ROS 2’s packages available for you to use in that terminal.
```bash 
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Creating a workspace
A workspace is a directory containing ROS 2 packages.
#### Creating a directory for the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
#### Build the workspace
 In the root of the project(i.e. ```~/ros2_ws```), run 
 
 ```bash
 colcon build
 ``` 

 Using build command with the tag ```--symlink-install``` allows the installed files to be changed by changing the files in the source space (e.g. Python files or other non-compiled resources) for faster iteration.

 After the build we should see directory structure as below:

```
├── build
├── install
├── log
└── src
```

#### Source the environment

colcon will bash/bat files in the ```install``` directory to help set up the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.

```bash
echo "source ~/ros2_ws/install/setup.bash " >> ~/.bashrc
```

#### Setup ```colcon``` tab completion

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

### Creating a package

A package is an organizational unit for your ROS 2 code. Package creation in ROS 2 uses ament as its build system and colcon as its build tool. A single workspace can contain as many packages as you want, each in their own folder.

Make sure you are in the ```src``` folder.

For Python
```bash
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy 
```
For CPP
```bash
ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp
```

