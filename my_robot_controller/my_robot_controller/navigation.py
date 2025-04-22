#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations
import math
import time


class TurtleNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("Navigation Node started")

        # Define goal positions and orientations (x, y, yaw in degrees)
        self.goal_poses = [
            {'x': 0.63, 'y': -2.11, 'yaw': -30},
            {'x': 4.49, 'y': -3.06, 'yaw': 60},
            {'x': 3.22, 'y': 0.11, 'yaw': 0},
            {'x': 7.24, 'y': -3.13, 'yaw': 90}
        ]

        self.current_goal_index = 0

        # Publishers
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10)

        # Subscriber to monitor the robot's current pose
        self.odom_listener = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)

        # Wait for everything to be initialized properly
        time.sleep(5)
        self.publish_initial_pose()

        time.sleep(5)
        self.publish_goal()

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'

        # Set initial position
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0

        # Convert yaw (heading) angle to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, 0)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]

        self.initial_pose_publisher.publish(initial_pose)

    def odom_callback(self, msg: Odometry):
        # Get current position from odometry
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]

        # Calculate distance to current goal
        distance_to_goal = math.sqrt(
            (current_pose.position.x - goal_pose['x']) ** 2 +
            (current_pose.position.y - goal_pose['y']) ** 2
        )

        # If the robot is close enough to the goal, move to the next
        if distance_to_goal < 0.3:
            self.publish_next_goal()

    def publish_next_goal(self):
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            self.publish_goal()
        else:
            self.get_logger().info("All goals reached!")
            rclpy.shutdown()

    def publish_goal(self):
        goal = self.goal_poses[self.current_goal_index]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'

        # Set goal position
        pose_msg.pose.position.x = goal['x']
        pose_msg.pose.position.y = goal['y']

        # Convert yaw (in degrees) to radians, then to quaternion
        quaternion = tf_transformations.quaternion_from_euler(
            0, 0, math.radians(goal['yaw']))
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Small delay before publishing
        time.sleep(0.5)
        self.goal_pose_publisher.publish(pose_msg)

        self.get_logger().info(f"Published goal {self.current_goal_index + 1}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation Node stopped")
    finally:
        rclpy.shutdown()
