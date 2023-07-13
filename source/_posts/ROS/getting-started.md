---
title: Getting started with ROS2 
tags: 
- Robotics
- ROS
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
For C++
```bash
ros2 pkg create <package_name> --build-type ament_cmake --dependencies rclcpp
```
### Building a package

You can build many packages at once by running colcon build in the workspace root.

```bash
cd ~/ros2_ws
colcon build
```
source the workspace bash file

```bash
source ~/.bashrc
```
To build only a particular package
```bash
colcon build --packages-select <package_name>
```

Package contents after building. Inside ```ros2_ws/src/<package_name>```, you will see the files and folders that ros2 pkg create automatically generated.

For Python
```bash
my_package  package.xml  resource  setup.cfg  setup.py  test
```
For C++
```bash
CMakeLists.txt  include  package.xml  src
```

## Nodes in ROS2
Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

<center>
<img src="https://docs.ros.org/en/humble/_images/Nodes-TopicandService.gif"  width="80%" height="80%">
</center>
 
[*source*](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#id3)

From now on I will be using python and ```<package_name>``` as ```my_package```.

Go to the folder inside your package which names as same as your package.(i.e.```my_package```)

```bash
cd  ros2_ws/src/my_package/my_package
```
### create a node which logs a message
Create a python file for you node and make it executable
```bash
touch my_first_node.py
chmod +x my_first_node.py
```

Write the first node as below.
```python
#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node") # node name that is used when we run the node in the graph
        self.get_logger().info("My first node message")

def main(args=None):
    rclpy.init(args=args) # initialize ros2 communication
    node = MyNode() # creating the node object
    rclpy.spin(node=node) # when you spin a node it is gonna be kept alive until you kills it.
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Install the node

Navigate to the ```setup.py``` in your package. Add a ```console script``` in the ```entry_points```. make the executable name ```test_node```

```python
entry_points={
        'console_scripts': [
            "test_node = my_package.my_first_node:main"
        ],
    },
```

### Build the package -- instructions above
```bash
cd ~/ros2_ws
colcon build
source ~/.bashrc
```
### Run the node
The command ```ros2 run``` launches an executable from a package. ```ros2 run <package_name> <executable_name>```

Run the below
```bash
ros2 run my_package test_node
```

It will output something like this
```[INFO] [1689263569.773487669] [first_node]: My first node message```

### Add a timer and a callback inside the node
We are going to log a message at every second.

```python
#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node") 
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode() 
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### ros2 node list

``ros2 node list`` will show you the names of all running nodes. 

this will output something like this:
```bash
/first_node
```
### ros2 node info

``ros2 node info /first_node``returns a list of subscribers, publishers, services, and actions.

The output should look like this:

```bash
/first_node
  Subscribers:

  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /first_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /first_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /first_node/get_parameters: rcl_interfaces/srv/GetParameters
    /first_node/list_parameters: rcl_interfaces/srv/ListParameters
    /first_node/set_parameters: rcl_interfaces/srv/SetParameters
    /first_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```
