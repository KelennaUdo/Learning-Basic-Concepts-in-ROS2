cd ~/ros2_ws

colcon build --packages-select my_py_pkg


# Source your workspace's setup script to include your package
source install/setup.bash

# Run the add_two_ints_server.py node from your package
# Replace 'my_py_pkg' with your actual package name if different
ros2 run my_py_pkg number_publisher