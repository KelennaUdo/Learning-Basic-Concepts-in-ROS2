#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwointsClientNode : public rclcpp::Node // MODIFY NAME
{
public:
    AddTwointsClientNode() : Node("add_two_ints_client") // MODIFY NAME
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void callAddTwoInts(int a, int b){
        while (!client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "waiting for server...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto callback = std::bind(&AddTwointsClientNode::callback_addTwoInts, this, std::placeholders::_1);
        client_->async_send_request(request, callback);
    }

private:
    void callback_addTwoInts(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
    }

    std::shared_ptr<rclcpp::Client<example_interfaces::srv::AddTwoInts>> client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwointsClientNode>(); // MODIFY NAME
    node->callAddTwoInts(6, 9);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
