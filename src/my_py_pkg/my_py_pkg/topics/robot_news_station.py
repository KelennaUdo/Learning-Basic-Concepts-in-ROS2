# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  
# from example_interfaces.msg import String 

class Robot_News_Station_Node(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_news_station") # MODIFY NAME
        self.publisher_ = self.create_publisher(String, 'robot_news', 10)
        self.news_message_ = "Robot Revolution Imminent"
        self.timer_ = self.create_timer(1.0, self.publish_news)  # 1 second interval

    def publish_news(self):
        msg = String()
        msg.data = self.news_message_
        self.publisher_.publish(msg)
        self.get_logger().info(f"Breaking news: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Robot_News_Station_Node() # MODIFY NAME
    node.get_logger().info("Robot News Station Node has started!") # MODIFY LOG MESSAGE
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
