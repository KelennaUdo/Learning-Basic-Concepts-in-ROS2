#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    # Create a new node named 'my_first_node'
    node = Node('my_first_node')
    # Log an info message to indicate the node has started
    node.get_logger().info('Hello, ROS 2!')
    
    try:
        # Keep the node running, processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # This block always runs, even if an exception occurs
        node.get_logger().info('Shutting down my_first_node...')
        # Destroy the node explicitly
        node.destroy_node() 
        # Shutdown the ROS 2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# End of my-first-node.py
# This is a simple ROS 2 node that logs a message when started and handles shutdown gracefully
# It can be run using the command: ros2 run my_py_pkg my_first_node
# Ensure that the package is built and sourced before running the node
