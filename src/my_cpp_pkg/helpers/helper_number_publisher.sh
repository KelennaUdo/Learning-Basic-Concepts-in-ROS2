#!/bin/bash
cd ~/ros2_ws/
colcon build --packages-select my_cpp_pkg
source install/setup.bash
ros2 run my_cpp_pkg number_publisher --ros-args --params-file ~/ros2_ws/yaml_param_files/number_params.yaml
