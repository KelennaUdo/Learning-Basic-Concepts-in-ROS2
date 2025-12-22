
cd ~/ros2_ws

colcon build --packages-select my_cpp_pkg

source install/setup.bash

ros2 run my_cpp_pkg robot_news_station
