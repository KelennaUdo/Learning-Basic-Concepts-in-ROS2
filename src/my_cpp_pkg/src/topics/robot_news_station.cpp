#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // ROS2 standard string message type

// Node class that publishes periodic "news" messages to a topic.
// Naming follows ROS2 Node conventions: <Something>Node.
class RobotNewsStationNode : public rclcpp::Node // MODIFY NAME
{
public:
    // Constructor takes the text message to repeatedly publish.
    // It also initializes the ROS2 node with the given node name.
    RobotNewsStationNode(std::string msg) : Node("robot_news_station") // MODIFY NAME
    {
        // Create a publisher that will publish std_msgs::msg::String messages
        // on the "robot_news" topic. The queue depth (history) is 10.
        publisher_ = this->create_publisher<std_msgs::msg::String>("robot_news", 10);

        // Create a timer that calls the provided lambda once every second.
        // The lambda captures 'msg' by value so the same message text is sent each tick.
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, msg]() { this->publishNews(msg); }
        );
    }

private:
    // Publisher handle used to actually send messages.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // Timer handle to keep the periodic callback alive.
    rclcpp::TimerBase::SharedPtr timer_;

    // Helper that constructs a std_msgs::msg::String message and publishes it.
    // Also logs the published text using the node's logger.
    void publishNews(std::string msg){
        // Allocate and populate the message.
        std::shared_ptr<std_msgs::msg::String> msg_ = std::make_shared<std_msgs::msg::String>();
        msg_->data = msg;

        // Publish the message to any subscribers on "robot_news".
        publisher_->publish(*msg_);

        // Informational log (printed to console by default).
        RCLCPP_INFO(this->get_logger(), "Publishing news: '%s'", msg_->data.c_str());
    }
};
    
int main(int argc, char **argv)
{
    // Initialize ROS2 client library.
    rclcpp::init(argc, argv);

    // Create the node instance on the heap and wrap in a shared_ptr.
    // The argument is the message text that will be repeatedly published.
    auto node = std::make_shared<RobotNewsStationNode>("The Robot Revolution has begun!"); // MODIFY NAME

    // Enter the ROS2 event loop; blocks until shutdown (e.g., Ctrl+C).
    rclcpp::spin(node);

    // Clean up and shutdown ROS2.
    rclcpp::shutdown();
    return 0;
}
