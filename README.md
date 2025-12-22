# Learning Basic Concepts in ROS 2

This repository captures my hands-on journey learning ROS 2, with small, focused examples of core ROS 2 concepts implemeted to build programs. It showcases clean, well-structured code while serving as both a personal reference and a growing portfolio of foundational robotics software skills.

## Sources

- ROS 2 for Beginners (ROS Jazzy – 2025) (Udemy)
- Introduction to ROS (DigiKey YouTube playlist)

## Workspace Layout

- `src/my_cpp_pkg`: C++ examples (nodes, topics, services)
- `src/my_py_pkg`: Python examples (nodes, topics, services)
- `src/my_robot_interfaces`: Custom message/service/interface definitions

## Concepts Covered

- Nodes: single-purpose processes that perform work and communicate.
- Topics: publish/subscribe message streams for data flow.
- Services: request/response calls for synchronous interactions.
- Interfaces: custom message/service definitions used across packages.
- Parameters: runtime configuration values for nodes.
- Launch files: a handy way to automate the running of many nodes with a single command

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
