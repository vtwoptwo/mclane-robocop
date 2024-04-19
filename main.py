#!/usr/bin/env python3

import rospy
import vera.computer_vision.cv_ws.src.detect_image.src.detect_image as detect_img
import juan_yu.src.lidar as lidar
import manu.camera_rgbd as follow
from sensor_msgs.msg import Image

rospy.init_node("main")

follower = follow.Follower()
obs_avoider = lidar.ObstacleAvoider() # juan: will handle continue moving in the main, should remove continue moving from the callback
img_processor = detect_img.ImageProcessor(
    "vera/computer_vision/cv_ws/src/detect_image/model/zane/train6/weights/best.pt"
)  # juan: can be set at the launch file

rospy.on_shutdown(img_processor.on_shutdown)

while not rospy.is_shutdown():

    if obs_avoider.obs_detected and not img_processor.img_detected:
        follower.image_sub.unregister() # so follower and obs_avoider won't fight for control
        while obs_avoider.obs_detected:
            print("Avoid obstacle...")
        follower.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, follower.image_callback
        )

    # if img_processor.img_detected:
    #     follower.image_sub.unregister()
    #     while img_processor.img_detected:
    #         rospy.loginfo("Process image...")
    #     follower.image_sub = rospy.Subscriber(
    #         "/camera/color/image_raw", Image, follower.image_callback
    #     )

    # rospy.sleep(0.1)
