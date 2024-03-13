#! /usr/bin/env python3

import rospy, time, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

detect_obj=False

def turn_left_to_avoid(range_data):
    rospy.loginfo(f'enter callback %f'%range_data.range)
    global detect_obj,sub_fl,sub_fr
    var = Twist()
    rospy.loginfo(f'turning %s'%detect_obj)
    if range_data.range<=0.2:
        sub_fl.unregister()
        #sub_fr.unregister()
        detect_obj=True
        var.angular.z = math.radians(90)
        end_time = time.time() + 1.75
        while time.time() < end_time:
            rospy.loginfo("Turn left...")
            pub.publish(var)
            rate.sleep()
        detect_obj=False
        sub_fl=rospy.Subscriber("/range/fl", Range, turn_left_to_avoid)
        #sub_fr=rospy.Subscriber("/range/fr", Range, turn_left_to_avoid)
    var.angular.z =0
    pub.publish(var)
    forward()


def pub_once():
    #var.angular.z = math.radians(90)
    var = Twist()
    i = 0
    while not i == 2:
        pub.publish(var)
        rospy.loginfo("Start after 2 seconds...")
        rospy.Rate(1).sleep()
        i += 1
    #var.angular.z =0
    #ub.publish(var)

def forward():
    var = Twist()
    rospy.loginfo("Go forward...")
    var.linear.x = 0.2
    pub.publish(var)
    
if __name__ == '__main__':
    global pub,sub_fl,sub_fr
    rospy.init_node('twist_publisher')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    sub_fl=rospy.Subscriber("/range/fl", Range, turn_left_to_avoid)
    #sub_fr=rospy.Subscriber("/range/fr", Range, turn_left_to_avoid)
    
    rate = rospy.Rate(10)
    pub_once()

    rospy.spin()