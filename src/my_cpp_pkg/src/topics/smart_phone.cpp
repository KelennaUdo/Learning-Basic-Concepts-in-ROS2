#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SmartPhoneNode : public rclcpp::Node // MODIFY NAME
{
public:
    SmartPhoneNode() : Node("smart_phone") // MODIFY NAME
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_news",
            10,
            std::bind(&SmartPhoneNode::callbackRobotNews, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Smart phone has been started (new).");
    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscriber_;

    void callbackRobotNews(const std_msgs::msg::String msg_){
        RCLCPP_INFO(this->get_logger(), "Subscribed to news: '%s'", msg_.data.c_str());
        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartPhoneNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
