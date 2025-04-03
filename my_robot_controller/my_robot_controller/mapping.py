#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()
        # Define the width of the range for obstacle detection
        a = 2  

        # Extract directional distances
        self._front = min(scan.ranges[:a+1] + scan.ranges[-a:])
        self._left = min(scan.ranges[90-a:90+a+1])
        self._right = min(scan.ranges[270-a:270+a+1])

        # Check for obstacle on the right or left side closer than 0.3m
        if self._left < 0.2:
            # Obstacle on the left, stop and rotate away (to the right)
            cmd.linear.x = 0.0
            cmd.angular.z = -0.4  # Rotate right
        elif self._right < 0.2:
            # Obstacle on the right, stop and rotate away (to the left)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.4  # Rotate left
        elif self._front < 1.2:  # Obstacle ahead
            if self._right < self._left:
                cmd.linear.x = 0.05
                cmd.angular.z = 0.8  # Turn left
            else:
                cmd.linear.x = 0.05
                cmd.angular.z = -0.8  # Turn right
        else:
            cmd.linear.x = 0.3  # Move forward
            cmd.angular.z = 0.0  # No turning

        # Publish the command
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()
