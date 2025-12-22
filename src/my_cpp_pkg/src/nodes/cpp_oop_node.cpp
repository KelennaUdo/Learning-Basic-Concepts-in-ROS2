#include "rclcpp/rclcpp.hpp"

// Define a class that inherits from rclcpp::Node
class MyNode : public rclcpp::Node
{
public:
    // Constructor: called when you create an instance of MyNode
    MyNode(const std::string & node_name): rclcpp::Node(node_name)
    {
        // Log a message when the node starts
        RCLCPP_INFO(this->get_logger(), "Hello world from my cpp node!");

        // Create a timer that triggers every 1 second
        // The timer calls the timer_callback() method each time it fires
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // Timer period: 1 second
            std::bind(&MyNode::timer_callback, this) // Bind the callback to 'this' instance
        ); 
    }
private:
    // This function is called every time the timer fires
    void timer_callback()
    {
        // Log the current value of counter_ each time the callback runs
        RCLCPP_INFO(this->get_logger(), "Hello %d from timer callback!", counter_);
        // Increment the counter for the next callback
        counter_++;
    }

    // Shared pointer to the timer object; keeps the timer alive as long as the node exists
    // rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    // Counter variable to demonstrate state across timer callbacks
    int counter_{0}; // Initialized to 0
};


// Main function: entry point of the program
int main(int argc, char* argv[]){
    // Initialize the ROS 2 C++ client library
    rclcpp::init(argc, argv);

    // Create a shared pointer to a new MyNode instance
    std::shared_ptr<MyNode> my_node = std::make_shared<MyNode>("cpp_node_name");

    // Spin the node: this keeps the node alive and processes callbacks (like the timer)
    rclcpp::spin(my_node);

    // Shutdown ROS 2 after node stops spinning (e.g., on signal interrupt)
    rclcpp::shutdown(); 

    return 0;
}