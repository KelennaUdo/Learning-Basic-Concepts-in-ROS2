cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(cpp_node src/nodes/cpp_oop_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/topics/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp std_msgs)

add_executable(smart_phone src/topics/smart_phone.cpp)
ament_target_dependencies(smart_phone rclcpp std_msgs)

add_executable(add_two_ints_server src/services/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/services/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/services/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hw_status_pub src/custom_interfaces/hardware_status_publisher)
ament_target_dependencies(hw_status_pub rclcpp my_robot_interfaces)

install(
  TARGETS cpp_node add_two_ints_server add_two_ints_client_no_oop add_two_ints_client smart_phone robot_news_station hw_status_pub
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
