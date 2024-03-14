#! /usr/bin/env python3

import rospy, time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

rospy.init_node('twist_publisher')
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rate = rospy.Rate(10)
var = Twist()
var.angular.z = math.radians(90)
print(var.angular.z)
end_time = time.time() + 1.82
while time.time() < end_time:
    rospy.loginfo("Turn left...")
    pub.publish(var)
    rate.sleep()
var.angular.z = 0
pub.publish(var)