#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
import cv2
from object_detection.srv import *
from sensor_msgs.msg import Image

obj_pub = rospy.Publisher('object', Image, queue_size=10)
mask_pub = rospy.Publisher('mask', Image, queue_size=10)

def predict_object(img_msg):
    rospy.wait_for_service("object_detection")
    try:
        predict_sv = rospy.ServiceProxy("object_detection", predict)
        service_request = predictRequest(img_msg,0)
        predict_result = predict_sv(service_request)
        return predict_result.output_mask

    except:
        rospy.loginfo ("Service is break")
        return

def main():
    rospy.init_node('object_detection_client')
    bridge = CvBridge()
    # img1 = cv2.imread("/home/michael/sis_mini_competition_2018/catkin_ws/src/competition_modules/object_detection/src/3.jpg",cv2.IMREAD_COLOR)
    # image_message = bridge.cv2_to_imgmsg(img1, encoding="bgr8")
    image_message = rospy.wait_for_message("/camera/rgb/image_color", Image)
    mask_message = predict_object(image_message)
    while not rospy.is_shutdown():
        obj_pub.publish(image_message)
        mask_pub.publish(mask_message)

if __name__ == "__main__":
    main()
