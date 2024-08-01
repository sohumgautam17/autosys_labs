#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.laser_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
    
        # TODO: Publish to drive
        self.driver_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges """
        safe_thresh = 2.0  # This is how far the obstacle can be for it to be safe
        start_indx = None
        counter = 0
        gaps = {}
        
        for i, range_val in enumerate(free_space_ranges):
            if range_val >= safe_thresh:
                if start_indx is None:
                    start_indx = i
                counter += 1
            else:
                if start_indx is not None:
                    gaps[start_indx] = start_indx + counter - 1
                    counter = 0
                    start_indx = None

        # Check if the last segment is a gap
        if start_indx is not None:
            gaps[start_indx] = start_indx + counter - 1

        max_length = 0
        max_gap_start = 0
        max_gap_end = 0
        max_total_dist = 0

        for start, end in gaps.items():
            length = end - start + 1
            total_dist = sum(free_space_ranges[start:end + 1])
            
            if length > max_length or (length == max_length and total_dist > max_total_dist):
                max_length = length
                max_total_dist = total_dist
                max_gap_start = start
                max_gap_end = end

        return max_gap_start, max_gap_end

    
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        best_point = start_i + ((end_i-start_i ) // 2.0)
        return best_point


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = np.array(data.ranges)
        # proc_ranges = self.preprocess_lidar(ranges)

        # Find max length gap
        start_i, end_i = self.find_max_gap(ranges)

        # Find the best point in the gap
        best_point = self.find_best_point(start_i, end_i, ranges)
        self.get_logger().info(f'The best point is {best_point} far away')

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = (best_point - len(ranges) // 2.0) * data.angle_increment

        if best_point < 550:
            drive_msg.drive.speed = 5.5 # You can adjust the speed accordingly
        else:
            drive_msg.drive.speed = 2.5 # You can adjust the speed accordingly

        self.driver_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
