# Learning Basic Concepts in ROS 2

This repository captures my hands-on journey learning ROS 2, with small, focused examples of core ROS 2 concepts implemeted to build programs. It showcases clean, well-structured code while serving as both a personal reference and a growing portfolio of foundational robotics software skills.

## Sources

I used the following learning resources

- ROS 2 for Beginners (ROS Jazzy – 2025) [Udemy](https://www.udemy.com/course/ros2-for-beginners/?couponCode=CM251222G1)
- Introduction to ROS [DigiKey YouTube playlist](https://www.youtube.com/playlist?list=PLEBQazB0HUySWueUF2zNyrA8LSX3rDvE7)
- ROS 2 Jazzy docs: [tutorials](https://docs.ros.org/en/jazzy/Tutorials.html), [basic concepts](https://docs.ros.org/en/jazzy/Concepts/Basic.html)

## Workspace Layout

- `src/my_cpp_pkg`: C++ examples (nodes, topics, services)
- `src/my_py_pkg`: Python examples (nodes, topics, services)
- `src/my_robot_interfaces`: Custom message/service/interface definitions
- `src/my_robot_bringup` : Dedicated package for launch files

## Concepts Covered

- Nodes: single-purpose processes that perform work and communicate.
    Examples:
        `src/my_py_pkg/my_py_pkg/nodes/my_first_node.py` and `src/my_cpp_pkg/src/nodes/cpp_node.cpp` creates a bare node and spins;
        `src/my_py_pkg/my_py_pkg/nodes/my_oop_node.py` and `src/my_cpp_pkg/src/nodes/cpp_oop_node.cpp` creates a similar node but used OOP and a timer object to print date periodically

- Topics: publish/subscribe message streams for data flow.
    Examples:
        `src/my_py_pkg/my_py_pkg/topics/robot_news_station.py` publishes `std_msgs/String` on  the `robot_news` topic;
        while `src/my_py_pkg/my_py_pkg/topics/smart_phone.py` subscribes and logs messages;
        `src/my_cpp_pkg/src/topics/robot_news_station.cpp` publishes news at 1 Hz; `src/my_cpp_pkg/src/topics/smart_phone.cpp` subscribes and logs.

- Services: request/response calls for synchronous interactions.
    Examples:
        `src/my_py_pkg/my_py_pkg/services/add_two_ints_server.py` serves a `AddTwoInts` response to an `AddTwoInts` client request; `src/my_py_pkg/my_py_pkg/services/add_two_ints_client.py` is the async client that makes the request; `src/my_py_pkg/my_py_pkg/services/add_two_ints_client_no_oop.py` does the same thing, but is implemented with procedural not OOP programming;
        `src/my_cpp_pkg/src/services/add_two_ints_server.cpp`, `src/my_cpp_pkg/src/services/add_two_ints_client.cpp`, and `src/my_cpp_pkg/src/services/add_two_ints_client_no_oop.cpp` are C++ equivalents.

- Interfaces: custom message/service definitions that can be used to communicate between nodes accross different packages.
    Examples:
        `src/my_py_pkg/my_py_pkg/custom_interfaces/hardware_status_publisher.py` publishes `my_robot_interfaces/HardwareStatus` (a custom message defined in the `my_robot_interfaces` package) at 2 Hz;
        `src/my_cpp_pkg/src/custom_interfaces/hardware_status_publisher.cpp` publishes the same custom message in C++.

- Parameters: runtime configuration values for nodes.
    Examples:
        `src/my_py_pkg/my_py_pkg/parameters/number_publisher.py` declares  two parameters to define publishing on a topic: `number` (what is published) and `timer_period` (how often it is published);
        `src/my_cpp_pkg/src/parameters/number_publisher.cpp` mirrors the same behavior in C++.

- Launch files: automate running multiple nodes with a single command.
    Example:
        `src/my_robot_bringup/launch/number_app.launch.py` launches the Python `number_publisher` and C++ `number_counter` together.

## Prerequisites

- ROS 2 Jazzy installed and sourced
- `colcon` and `rosdep`

## Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run Examples

```bash
# Python nodes
ros2 run my_py_pkg <node_name>

# C++ nodes
ros2 run my_cpp_pkg <node_name>
```
