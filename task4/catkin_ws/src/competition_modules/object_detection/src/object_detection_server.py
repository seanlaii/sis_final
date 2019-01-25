#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from object_detection.srv import *

class Object_detection_server():
    def __init__(self):
        self.srv = rospy.Service("object_detection", predict, self.handle_object_detection)


    def handle_object_detection(self, req):
        rospy.loginfo("Test")

        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(req.input_image, desired_encoding="bgr8")
        if req.dataset_id == 0:
            mask = self.detect_geometric_object(image)
            mask = np.array(mask,dtype=np.uint8)
            image_message = bridge.cv2_to_imgmsg(mask, encoding="bgr8")
            rospy.loginfo ("Finish object detection ")
            return predictResponse(image_message)
        else:
            self.detect_product(image)


    def detect_geometric_object(self, image):
        rospy.loginfo("Predict deometric objects")
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # define range of red color in HSV
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])

        lower_blue = np.array([80,50,50])
        upper_blue = np.array([130,255,255])

        lower_green = np.array([30,80,40])
        upper_green = np.array([80,255,255])

        object_mask = np.zeros(np.shape(hsv_image),dtype=np.int8)
        red_mask = self.hsv_filter(hsv_image, lower_red, upper_red)
        green_mask = self.hsv_filter(hsv_image, lower_green, upper_green)
        blue_mask = self.hsv_filter(hsv_image, lower_blue, upper_blue)

        reg_mask = np.zeros(np.shape(red_mask),dtype=np.int8)
        reg_mask[red_mask>0] = 255
        object_mask[:,:,2] = reg_mask

        reg_mask = np.zeros(np.shape(green_mask),dtype=np.int8)
        reg_mask[green_mask>0] = 255
        object_mask[:,:,1] = green_mask

        reg_mask = np.zeros(np.shape(blue_mask),dtype=np.int8)
        reg_mask[blue_mask>0] = 255
        object_mask[:,:,0] = blue_mask
        return object_mask


    def detect_product(self, image):
        rospy.loginfo("Predict products")


    def hsv_filter(self, img, lower_bd, upper_bd):
        mask = cv2.inRange(img, lower_bd, upper_bd)
        mask_eroded = cv2.erode(mask, None, iterations = 3)
        mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 3)
        return mask_eroded_dilated


def main():
    rospy.init_node('object_detection_server')
    object_detection_server = Object_detection_server()
    rospy.loginfo("Object detection server is ready")
    rospy.spin()

if __name__ == "__main__":
    main()
