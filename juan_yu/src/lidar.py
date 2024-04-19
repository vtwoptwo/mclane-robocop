#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider:

    def __init__(self):
        rospy.init_node("scan_values")
        self.pub = rospy.self.publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.move_cmd = Twist()
        self.obs_detected = False  # juan: flag for main to stop the robot
        rospy.spin()

    def callback(self, msg):

        # Get the range data from the laser scan
        ranges = msg.ranges

        # Define the threshold distance (in meters)
        threshold_distance = 0.2

        # Check each range reading against the threshold distance
        for range_reading in ranges:
            if range_reading < threshold_distance:
                self.obs_detected = True
                # Stop the movement
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0
                # self.publish the stop command
                self.pub.self.publish(self.move_cmd)
                self.obs_detected = False
                return


if __name__ == "__main__":
    try:
        ObstacleAvoider()
    except rospy.ROSInterruptException:
        pass
