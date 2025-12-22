# This is a helper script to automate the setup process for a ROS 2 Python package.
# run:
# chmod +x helper.sh
# in the my_py_pkg directory to automate the setup process.



cd ~/ros2_ws


colcon build --packages-select my_py_pkg

# Source the workspace to make the package available
# This is necessary to ensure that the newly built package is recognized by ROS 2.
# If you are using a different shell, adjust the source command accordingly.
# For example, if you are using zsh, use `source install/setup.zsh`.
# If you are using bash, use `source install/setup.bash`.
source install/setup.bash

# To run the Python node, use the following command:
# Make sure you have the correct package name and node name.
# Replace 'my_py_pkg' with your package name and 'smart_phone' with your node name according to setup.py.
ros2 run my_py_pkg smart_phone



# Recommended Directory Structure for ROS 2 Python Package
# This structure is commonly used for ROS 2 Python packages, ensuring that the package is well-organized and follows best practices.

# ~/ros2_ws/
#  ├── src/
#  │   └── my_package/
#  │        ├── package.xml
#  │        ├── setup.py / CMakeLists.txt  (depending on Python or C++)
#  │        ├── my_package/
#  │        │    ├── __init__.py
#  │        │    └── my_node.py
#  │        └── resource/
#  ├── build/
#  ├── install/
#  ├── log/



# This is a recommended directory structure for a ROS 2 Python package on github.
# ros2_ws/
#  ├── .devcontainer/          (optional, if you use VS Code Remote)
#  ├── src/
#  │   └── my_package/
#  │        ├── package.xml
#  │        ├── setup.py
#  │        ├── my_package/
#  │             ├── __init__.py
#  │             └── my_node.py
#  │        └── launch/
#  │             └── my_launch.py
#  ├── run_node.sh
#  ├── README.md

