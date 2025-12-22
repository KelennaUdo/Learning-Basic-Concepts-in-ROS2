#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <sstream>

class AddTwoIntsServerNode : public rclcpp::Node // MODIFY NAME
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server") // MODIFY NAME
    {
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", 
            std::bind(&AddTwoIntsServerNode::callbackAddTwoInts,this, std::placeholders::_1, std::placeholders::_2)
        );
            RCLCPP_INFO(this->get_logger(), "AddTwoInts service server has been initialized (new).");
    }

private:
    void callbackAddTwoInts(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                            const  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
                            {
                                response->sum = request->a + request->b;
                                std::ostringstream output;
                                output << request->a << " + " << request->b << " = " << response->sum;
                                RCLCPP_INFO(this->get_logger(), "%s", output.str().c_str());            
                            }

    std::shared_ptr<rclcpp::Service<example_interfaces::srv::AddTwoInts>> server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
