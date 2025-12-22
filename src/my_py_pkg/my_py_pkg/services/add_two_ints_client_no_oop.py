#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")
    
    # Create a client for the AddTwoInts service
    client = node.create_client(AddTwoInts, "add_two_ints")
    # Wait until the service is available
    while not client.wait_for_service(1):
        node.get_logger().warn("Waiting for add_two_ints service!")

    # Create a request and set the values to add
    request = AddTwoInts.Request()
    request.a = 11
    request.b = 373

    # Call the service asynchronously
    future = client.call_async(request)
    # Wait for the service call to complete
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    # Check if the response is valid and log the result
    if response is not None:
        node.get_logger().info(f'Result: {response.sum}')
    else:
        node.get_logger().error('Service call failed')

    rclpy.shutdown()


if __name__ == "__main__":
    main()
