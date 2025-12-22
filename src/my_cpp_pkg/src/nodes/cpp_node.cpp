#include "rclcpp/rclcpp.hpp"



int main(int argc, char* argv[]){
// Initialize the ROS 2 C++ client library
    rclcpp::init(argc, argv);

// Create a shared pointer to a new Node instance
    std::shared_ptr<rclcpp::Node> my_node = std::make_shared<rclcpp::Node>("cpp_node_name");
    // Alternative way to create the node (same name):
//  auto my_node = std::make_shared<rclcpp::Node>("cpp_node_name");
// Log a message using the node's logger
    RCLCPP_INFO(my_node->get_logger(), "hello world from my cpp node!");
// spin the node to process callbacks and keep it alive
    rclcpp::spin(my_node);
// Shutdown ROS 2 after node stops spinning (e.g., on signal interrupt)
    rclcpp::shutdown(); 

    return 0;
}