#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """Wall Following Node for autonomous driving"""

    def __init__(self):
        super().__init__('wall_follow_node')

        # Parameters
        self.kp = self.declare_parameter('kp', 1.0).get_parameter_value().double_value
        self.ki = self.declare_parameter('ki', 0.001).get_parameter_value().double_value
        self.kd = self.declare_parameter('kd', 0.005).get_parameter_value().double_value
        self.desired_distance = self.declare_parameter('desired_distance', 1.0).get_parameter_value().double_value
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 1.0).get_parameter_value().double_value

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 
        self.velocity = 2.0

        self.integral = 0.0
        self.prev_error = 0.001
        self.prev_time = time.time()

        # Create subscribers and publishers
        self.laser_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.get_logger().info('Wall Following Node Initialized')

    def get_range(self, range_data, angle):
        """Retrieve range at a specific angle from the LaserScan data."""
        angle_rad = np.radians(angle)
        index = int((angle_rad - range_data.angle_min) / range_data.angle_increment)
        
        if index < 0 or index >= len(range_data.ranges):
            return float('inf')
        
        range_value = range_data.ranges[index]
        return float('inf') if np.isnan(range_value) or np.isinf(range_value) else range_value

    def calculate_error(self, range_data):
        """Compute the error based on current distance and desired distance."""
        angle_b = 35  # This is making an acute angle
        angle_a = 90  # This is directly to the left

        dist_nw = self.get_range(range_data, angle_b)
        dist_left = self.get_range(range_data, angle_a)

        # Debug logging for range values
        # self.get_logger().info(f"dist_nw: {dist_nw}, dist_left: {dist_left}")

        if np.isinf(dist_nw) or np.isinf(dist_left):
            # self.get_logger().warn('Invalid range data detected, returning error 0.0')
            return 0.0

        theta = np.radians(abs(angle_b - angle_a))

        # Add a check for valid theta
        if theta == 0:
            # self.get_logger().warn('Theta is zero, returning error 0.0')
            return 0.0

        # dist_left in the latter part of equation may be dist_nw
        alpha = np.arctan2(dist_nw * np.cos(theta) - dist_left, (dist_nw) * np.sin(theta))

        # Add logging to check alpha value
        # self.get_logger().info(f"alpha: {alpha}")

        current_distance = dist_left * np.cos(alpha)  # dist_nw may also be mixed up with dist_left
        future_distance = current_distance + (self.lookahead_distance * np.sin(alpha))  # lookahead distance is car length in other repo

        error = self.desired_distance - future_distance  # desired dist from wall - future distance

        # Add logging to check current and future distance
        # self.get_logger().info(f"current_distance: {current_distance}, future_distance: {future_distance}")

        return error

    def pid_control(self, error):
        """Compute the control signal using PID control."""
        current_time = time.time()
        delta_time = current_time - self.prev_time

        if delta_time == 0:
            delta_time = 1e-6
        
        self.integral += error * delta_time

        # Check for invalid values before calculating derivative
        if np.isnan(error) or np.isnan(self.prev_error):
            self.get_logger().warn('Invalid error value detected, skipping PID calculation.')
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / delta_time

        control_signal = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        self.prev_time = current_time
        
        return control_signal


    def scan_callback(self, range_data):
        """Callback function for processing LaserScan data."""
        # Check for valid LaserScan data
        if not range_data.ranges:
            self.get_logger().warn('Received empty LaserScan data')
            return

        error = self.calculate_error(range_data)
        steering_angle = self.pid_control(error) # this is angle in other repo
        # steering_angle = np.clip(steering_angle, -1.0, 1.0)  # Limit the steering angle

        speed = self.calculate_speed(steering_angle, self.velocity)

        # Debug logging
        # self.get_logger().info(f"Error: {error}, Steering Angle: {steering_angle}, Speed: {speed}")

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = -steering_angle  # In other repo they made steering angle minus
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def calculate_speed(self, steering_angle, velocity):
        """Determine the speed based on the steering angle."""
        abs_angle = abs(steering_angle)
        if abs_angle < np.radians(10):
            return self.velocity
        elif abs_angle < np.radians(20):
            return 1.0
        else:
            return 0.75

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    
    # Clean up
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
