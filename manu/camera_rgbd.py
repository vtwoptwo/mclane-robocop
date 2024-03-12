#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cv_image = None
        self.depth_image = None
        self.depth_scale = None
        self.camera_info = None

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('Camera Image', self.cv_image)
        cv2.waitKey(1)  # Wait 1 millisecond for a key press

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)
        self.depth_scale = msg.depth_scale

    def process_image(self):
        if self.cv_image is None or self.depth_image is None:
            return

        # Detect the white line on the yellow floor (color segmentatin)
        line_contours = self.detect_white_line(self.cv_image)

        # Possibly I'll have to determine the position of the line relative to the robot's position using depth information
        # I may need to transform the depth image to match the RGB image (TF)

        # Control algorithm to make the robot follow the detected line
        self.follow_line(line_contours)

    def detect_white_line(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
       
        # Define lower and upper bounds for white color
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
       
        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
       
        # Apply morphological operations to remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
       
        # Find contours of white objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
        return contours

    def follow_line(self, line_contours):
        # Calculate center of the image
        image_center = self.cv_image.shape[1] // 2
       
        # Initialize error
        error = 0
       
        if len(line_contours) > 0:
            # Get bounding box of the largest contour
            x, y, w, h = cv2.boundingRect(line_contours[0])
           
            # Calculate center of the detected line
            line_center = x + (w // 2)
           
            # Calculate error
            error = line_center - image_center
       
        # Define proportional gain
        Kp = 0.01
       
        # Calculate control input (angular velocity)
        angular_velocity = Kp * error
       
        # Publish velocity commands using Twist message
        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        print ('hola')
        
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.process_image()
            rate.sleep()

if __name__ == '__main__':
    try:
        lf = LineFollower()
        lf.run()
    except rospy.ROSInterruptException:
        pass