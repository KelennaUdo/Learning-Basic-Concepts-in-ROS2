#!/bin/bash
cd ~/ros2_ws/
colcon build --packages-select my_cpp_pkg
source install/setup.bash
ros2 run my_cpp_pkg add_two_ints_server
