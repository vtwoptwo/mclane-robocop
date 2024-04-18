#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.turn_direction = 1

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

        lower_white = np.array([0, 0, 150]) #white
        upper_white = np.array([180, 50, 255]) # white
        mask = cv2.inRange(hsv, lower_white, upper_white)
        cv2.imwrite('output_image.jpg', mask)
        
        h, w, d = image.shape
        search_bot = 3 * h // 4  # Bottom quarter starts at three-fourths of the image height
        mask[0:search_bot, 0:w] = 0  # Mask out everything above the bottom quarter
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # Proportional controller (P-controller)
            center = w / 2
            max_ang_vel = 0.3  # Maximum angular velocity
            min_ang_vel = -0.3  # Minimum angular velocity
            Kp = 0.01  # Proportional gain
            error = cx - center
            ang_vel = np.clip(-Kp * error, min_ang_vel, max_ang_vel)
            self.twist.linear.x = 0.1
            self.twist.angular.z = ang_vel
            print(ang_vel)
            if ang_vel > 0:
                self.turn_direction = 1
            else:
                self.turn_direction = -1
            self.cmd_vel_pub.publish(self.twist)
        else:
            # If the line is not detected, start turning in the last known direction
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3 * self.turn_direction  # Turn left or right
            self.cmd_vel_pub.publish(self.twist)

rospy.init_node('follower')
follower = Follower()
rospy.spin()


# left >0
# right <0
