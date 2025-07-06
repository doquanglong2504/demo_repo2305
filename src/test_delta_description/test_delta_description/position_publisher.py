#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math
import numpy as np

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.publisher = self.create_publisher(Point, 'target_position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0.0
        self.center = np.array([0.0, 0.0, -0.5])  # Center position
        self.radius = 0.1  # Radius of circular motion
        self.height = -0.5  # Base Z position
        self.speed = 0.5   # Speed of motion

    def timer_callback(self):
        msg = Point()
        
        # Circular trajectory in XY plane
        msg.x = self.center[0] + self.radius * math.cos(self.time)
        msg.y = self.center[1] + self.radius * math.sin(self.time)
        
        # Add vertical motion (sine wave)
        vertical_amp = 0.05  # 5 cm amplitude
        msg.z = self.height + vertical_amp * math.sin(2 * self.time)
        
        self.publisher.publish(msg)
        self.time += self.speed * 0.1
        self.get_logger().info(f"Published target: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
