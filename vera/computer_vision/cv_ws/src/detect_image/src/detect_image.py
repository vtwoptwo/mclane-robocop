#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

class ImageProcessor:
    def __init__(self, model_path):
        self.node_name = "detect_image"
        rospy.init_node(self.node_name)
        
        self.model = YOLO(model_path)
        self.subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
       	self.publisher = rospy.Publisher("/camera/boxes", Image, queue_size=1)
        self.bridge = CvBridge()

        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        results = self.model.predict(cv_image)
        for result in results:
            import pdb;pdb.set_trace()
            annotated_frame = result.plot()   
            probs = result.boxes.conf #example output if something is detected:tensor([0.9451, 0.9358])
            cl = result.boxes.cls # tensor([4., 4.])
            # if the tensors of probs and cls exist
            
                    
            try:
                ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.publisher.publish(ros_image) 
            except CvBridgeError as e:
                print(e)

        # Publish the processed image
        #for detection in results:
        #cv2.imshow('YOLO Object Detection', cv_image)
        #cv2.waitKey(3)
    def on_shutdown(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    model_path = rospy.get_param('detect_image/model_file','')
    print(model_path)
    image_processor = ImageProcessor(model_path)
    rospy.on_shutdown(image_processor.on_shutdown)

