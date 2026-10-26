#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pathlib import Path

# Import your pure Python class
from data_collection_pkg.session import TimestampedRecorder

class SessionNode(Node):
    def __init__(self):
        super().__init__('session_node')
        
        # Declare parameters based on TimestampedRecorder init
        self.declare_parameter('output_root', '/tmp/sessions')
        self.declare_parameter('marker_length_m', 0.05)
        self.declare_parameter('apriltag_family', 'tag36h11')
        self.declare_parameter('detect_tags', False)
        
        output_root = Path(self.get_parameter('output_root').value)
        marker_length = self.get_parameter('marker_length_m').value
        family = self.get_parameter('apriltag_family').value
        detect = self.get_parameter('detect_tags').value
        
        self.recorder = TimestampedRecorder(
            output_root=output_root,
            marker_length_m=marker_length,
            apriltag_family=family,
            detect_tags=detect
        )
        self.get_logger().info(f"Session Recorder initialized at {output_root}")

        # Start the recording loop
        # Note: In a pure ROS2 design, you might trigger this via a Service call 
        # rather than running it instantly on startup.
        try:
            self.get_logger().info("Starting recording...")
            self.recorder.record() # This will block until stopped
        except Exception as e:
            self.get_logger().error(f"Recording error: {e}")

    def destroy_node(self):
        self.recorder.stop()[cite: 6]
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SessionNode()
    # No need to spin if recorder.record() is blocking, but included for standard ROS2 structure
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()