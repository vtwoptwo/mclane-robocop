#! /usr/bin/env python3

import rospy, time, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoider:

    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.move_cmd = Twist()
        self.obs_detected = False  # juan: flag for main to stop the robot
        rospy.spin()

    def callback(self, msg):

        # Get the range data from the laser scan
        ranges = msg.ranges

        # Define the threshold distance (in meters)
        threshold_distance = 0.2
        front=ranges[0:1]#[:122]+ranges[-122:] #5.41 indexes is 1 degree
        right=ranges[486:487]#[243:730]
        left=ranges[-486:-487]#[-730:-243]
        # Check each range reading against the threshold distance
        for range_reading in front:
            if range_reading < threshold_distance:
                self.obs_detected = True
                # Stop the movement
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.0
                # self.publish the stop command
                self.pub.publish(self.move_cmd)
                self.obs_detected = False
                return
        for range_reading in right:
            if range_reading < threshold_distance:
                self.obs_detected = True
                self.move_cmd.angular.z = math.radians(-30)
                end_time = time.time() + 1
                self.sub.unregister()
                while time.time() < end_time:
                    rospy.loginfo("Turn left...")
                    self.pub.publish(self.move_cmd)
                self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
                self.obs_detected = False
                return
        for range_reading in left:
            if range_reading < threshold_distance:
                self.obs_detected = True
                self.move_cmd.angular.z = math.radians(30)
                end_time = time.time() + 1
                self.sub.unregister()
                while time.time() < end_time:
                    rospy.loginfo("Turn right...")
                    self.pub.publish(self.move_cmd)
                self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
                self.obs_detected = False
                return


if __name__ == "__main__":
    try:
        rospy.init_node("scan_values")
        ObstacleAvoider()
    except rospy.ROSInterruptException:
        pass
