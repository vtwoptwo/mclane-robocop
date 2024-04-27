#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from ultralytics import YOLO


class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.ir_fl_sub = rospy.Subscriber('/range/fl', Range, self.ir_callback)  # Front-left IR sensor
        self.ir_fr_sub = rospy.Subscriber('/range/fr', Range, self.ir_callback)  # Front-right IR sensor

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.box_pub = rospy.Publisher('/camera/boxes', Image, queue_size=1)

        # CV Bridge
        self.bridge = cv_bridge.CvBridge()

        # Image processing
        self.model = YOLO('/home/v/mclane-robocop/catkin_ws/src/mclane_robocop/model/weights/best.pt')
        self.twist = Twist()

        # State flags
        self.turn_direction = 1
        self.obs_detected = False
        self.img_detected = False
        self.ir_detected = False

    def ir_callback(self, msg):
        # Assuming a minimum detection range that indicates an obstacle
        if msg.range < 0.05:  # 20 cm as an example threshold
            self.ir_detected = True
            self.backup_and_reset()

    def backup_and_reset(self):
        # Move backward for 2 seconds
        self.twist.linear.x = -0.1
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(2)  # Robot moves backward for 2 seconds

        # Stop and clear the IR detection flag
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.ir_detected = False

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
            return

        # Line following logic
        self.process_line_following(cv_image)

        # Object detection logic
        results = self.model.predict(cv_image)
        self.process_object_detection(results, cv_image)

    def process_object_detection(self, results, image):
        for result in results:
            probs = result.boxes.conf
            cl = result.boxes.cls
            probs = probs.cpu()
            cl = cl.cpu()

            # Convert to numpy array
            probs_numpy = probs.numpy()
            cl_numpy = cl.numpy()

            # Check if the array is not empty and all elements are greater than 0.9
            if probs_numpy.size > 0 and np.all(probs_numpy > 0.9):
                print("All probabilities are greater than 0.9")
                self.img_detected = True
                print("Detected high confidence object")
                annotated_frame = result.plot()
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                    self.box_pub.publish(ros_image)
                except cv_bridge.CvBridgeError as e:
                    print(e)
                        

    def process_line_following(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        h, w, _ = image.shape
        search_bot = 3 * h // 4
        mask[0:search_bot, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            center = w / 2
            error = cx - center
            angular_z = np.clip(-0.01 * error, -0.3, 0.3)
        else:
            # Default turn if no line is detected
            angular_z = 0.3 * self.turn_direction

        # Only modify linear.x if no obstacle is detected
        if not self.obs_detected:
            self.twist.linear.x = 0.1
        self.twist.angular.z = angular_z

        self.cmd_vel_pub.publish(self.twist)

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        threshold_distance = 0.2

        # Find if there are any close obstacles within the threshold
        if np.any(ranges < threshold_distance):
            self.obs_detected = True
            # Find the minimum range and its index
            min_index = np.argmin(ranges)
            n_ranges = len(ranges)

            # Determine direction: if the obstacle is in the first half of ranges, turn right; else, turn left
            if min_index < n_ranges / 2:
                # Obstacle on the left side
                self.twist.angular.z = -0.3  # Smaller turn rate to the right
            else:
                # Obstacle on the right side
                self.twist.angular.z = 0.3  # Smaller turn rate to the left
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.obs_detected = False

    def run(self):
        # Set the loop rate (in Hz, i.e., times per second)
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    rc = RobotController()
    rc.run()

