#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Smart_Phone_Node(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("smart_phone") # MODIFY NAME
        self.subscriber_ = self.create_subscription(String, "robot_news", self.smart_phone_callback, 10)
       

    def smart_phone_callback(self, msg: String):
        self.get_logger().info(f"Received message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Smart_Phone_Node() # MODIFY NAME
    node.get_logger().info("smart_phone node is activated!")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
