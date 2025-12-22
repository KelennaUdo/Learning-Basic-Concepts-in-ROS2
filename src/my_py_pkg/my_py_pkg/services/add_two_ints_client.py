#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # wait for server (can also be added as the first code in send_request())
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for add_two_ints service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_add_two_ints_response)
        

    def callback_add_two_ints_response(self, future):
        repsonse = future.result()
        self.get_logger().info(f"got response: {repsonse.sum}")

def main(args=None):
    rclpy.init(args=args)
 
    a = 2
    b = 3
    node = AddTwoIntsClientNode()
    node.send_request(a, b)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
