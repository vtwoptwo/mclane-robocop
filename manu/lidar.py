#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Global variable to hold the movement command
move_cmd = Twist()

def callback(msg):
    global move_cmd
    
    # Get the range data from the laser scan
    ranges = msg.ranges
    
    # Define the threshold distance (in meters)
    threshold_distance = 0.2
    
    # Check each range reading against the threshold distance
    for range_reading in ranges:
        if range_reading < threshold_distance:
            # Stop the movement
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            # Publish the stop command
            pub.publish(move_cmd)
            return
    
    # If no object is detected, continue moving
    move_cmd.linear.x = 0.1  # Adjust linear velocity as needed
    move_cmd.angular.z = 0.0  # Adjust angular velocity as needed
    pub.publish(move_cmd)

def stop_movement():
    global pub
    rospy.init_node('scan_values')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        stop_movement()
    except rospy.ROSInterruptException:
        pass
