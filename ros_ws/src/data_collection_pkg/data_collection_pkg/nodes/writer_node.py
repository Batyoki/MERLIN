#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pathlib import Path

# Import your pure Python classes
from data_collection_pkg.writer import FrameWriter, JsonlStore

class WriterNode(Node):
    def __init__(self):
        super().__init__('writer_node')
        
        # Declare parameters based on FrameWriter[cite: 4]
        self.declare_parameter('max_queue_size', 512)
        queue_size = self.get_parameter('max_queue_size').value
        
        # Initialize the async writer[cite: 4]
        self.writer = FrameWriter(max_queue=queue_size)
        self.get_logger().info("Async FrameWriter initialized")

        # TODO: You would add a subscriber here that listens for image messages 
        # over ROS2, and calls self.writer.write(path, image) when received[cite: 4].

    def destroy_node(self):
        # Ensure the queue flushes before the node dies[cite: 4]
        self.get_logger().info("Flushing writer queue to disk...")
        self.writer.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WriterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()