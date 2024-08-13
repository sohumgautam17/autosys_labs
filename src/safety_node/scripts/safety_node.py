#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# Import needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.ack_drive_speed_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.odom_subscription = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10) # Receive current speed of the vehicle

        self.laser_scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.min_speed = 0.0
        self.threshold = 0.85
        self.max_speed = 2.0

        self.current_speed = 0.0

        self.get_logger().info('Node is running')

    def odom_callback(self, odom_msg):
        self.current_speed = odom_msg.twist.twist.linear.x
        # self.get_logger().info(f'Current speed updated: {self.current_speed}')

    def scan_callback(self, scan_msg):
        # self.get_logger().info('Received laser scan message')
        ranges = scan_msg.ranges
        inst_ttc = self.calculate_ittc(ranges, self.current_speed)

        # self.get_logger().info(f'Instantaneous TTC values: {inst_ttc}')

        for ittc in inst_ttc:
            if np.isinf(ittc) or ittc < self.threshold:
                # self.get_logger().info(f'TTC below threshold: {ittc}')
                self.publish_brake_command()
                break

    def calculate_ittc(self, ranges, current_speed):
        ittcs = []
        for r in ranges:
            if r == 0.0:
                ittcs.append(float('inf'))
            else:
                range_rate = current_speed / r
                ittc = r / max(range_rate, 0.01)  # This outputs how much time left till collision
                ittcs.append(ittc)
        return ittcs

    def publish_brake_command(self):
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = self.min_speed
        self.ack_drive_speed_pub.publish(brake_msg)
        self.get_logger().info('Emergency Brake Activated')


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
