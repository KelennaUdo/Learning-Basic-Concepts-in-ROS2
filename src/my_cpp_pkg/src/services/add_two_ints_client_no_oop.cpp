#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <sstream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop"); // MODIFY NAME

    auto client = node->create_client<example_interfaces::srv::AddTwoInts> ("add_two_ints");
    while (!client->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_WARN(node->get_logger(), "waiting for server...");
    }

    // Send a request
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 2;
    request->b = 3;
    auto result = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);

    auto response = result.get();

    std::ostringstream output;
    output << request->a << " + " << request->b << " = " << response->sum;
    RCLCPP_INFO(node->get_logger(), "%s", output.str().c_str());

    rclcpp::shutdown();
    return 0;
}
