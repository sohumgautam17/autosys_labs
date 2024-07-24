#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscribers and publishers
        self.laser_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # PID control parameters
        self.kp = 12.0
        self.kd = 0.7
        self.ki = 6.0

        self.dist_from_wall = 0.1  # Desired distance from the wall
        self.integral = 0.0
        self.prev_error = 0.0
        self.previous_time = time.time()

        self.get_logger().info('Wall Following Node Initialized')

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """
        # Convert angle to index
        angle_rad = np.radians(angle)
        index = int((angle_rad - range_data.angle_min) / range_data.angle_increment) 

        # Handle out-of-bounds index
        if index < 0 or index >= len(range_data.ranges):
            return float('inf')
        
        # Handle NaNs and infs
        if np.isnan(range_data.ranges[index]) or np.isinf(range_data.ranges[index]):
            return float('inf')
        else:
            return range_data.ranges[index]

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left.
        
        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        angle_left = 90  # Angle for left wall
        angle_right = -45  # Angle for right wall

        dist_left = self.get_range(range_data, angle_left)
        dist_right = self.get_range(range_data, angle_right)

        if dist_left < dist_right:
            error = dist - dist_left
        else:
            error = dist_right - dist
        
        return error

    def pid_control(self, error): # I removed velocity
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        current_time = time.time()
        delta_time = current_time - self.previous_time
        
        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time
        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        self.previous_time = current_time
        
        # Calculate steering angle
        steering_angle = control_output
        
        # Set speed based on steering angle
        # speed = 2.0 if abs(steering_angle) < 0.5 else 1.0
        
        # # Publish drive message
        # drive_msg = AckermannDriveStamped()
        # drive_msg.drive.steering_angle = steering_angle
        # drive_msg.drive.speed = speed
        # self.drive_pub.publish(drive_msg)

    def check_for_opening(self, range_data):
        """
        Check for an opening to the left of the car.
        
        Args:
            range_data: single range array from the LiDAR

        Returns:
            is_opening: True if there is an opening to the left, False otherwise
        """
        left_opening_angle = 75  # Angle to check for left opening
        opening_distance_threshold = 1.3  # Distance threshold to consider as an opening

        dist_left = self.get_range(range_data, left_opening_angle)
        return dist_left > opening_distance_threshold
    

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        drive_msg = AckermannDriveStamped()
        if self.check_for_opening(msg):
            # If there is an opening to the left, turn left
            drive_msg.drive.steering_angle = 1.0  # Turn left
            drive_msg.drive.speed = 0.5  # Slow down while turning
            self.drive_pub.publish(drive_msg)
        else:
            # Calculate the error and control the car
            # drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 1.5
            # self.get_logger().info(f"Speed of car is {drive_msg.drive.speed}")
            self.drive_pub.publish(drive_msg)

            error = self.get_error(msg, self.dist_from_wall)
            self.pid_control(error)
        

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
