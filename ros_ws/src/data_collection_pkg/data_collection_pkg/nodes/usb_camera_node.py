#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import your pure Python class from the package directory
from data_collection_pkg.usb import CameraStream 

class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Declare parameters (matching what you put in the launch file)
        self.declare_parameter('device_index', 0)
        device_idx = self.get_parameter('device_index').value
        
        # Initialize your pure Python class
        try:
            self.stream = CameraStream(index=device_idx)
            self.get_logger().info(f"Successfully connected to camera {device_idx}")
        except Exception as e:
            self.get_logger().error(f"Failed to start camera: {e}")

        # You would typically set up a ROS2 timer here to constantly 
        # call self.stream.read() and publish the frames to a ROS topic.

def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()