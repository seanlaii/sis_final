#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
import cv2
import os
from sensor_msgs.msg import Image
def extract_data(period=1.5, save_folder="/home/michael/data/scene_0000", start_id=0):
    bridge = CvBridge()
    id = start_id
    while not rospy.is_shutdown():
        img_msg = rospy.wait_for_message('/camera/color/image_rect_color', Image)
        image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        id = id + 1
        id_str = "%.6d" % id
        file_name = "frame-" + id_str + ".jpg"
        # print file_name
        save_path = os.path.join(save_folder,file_name)
        print save_path
        cv2.imwrite(save_path, image)
        rospy.sleep(period)
def main():
    print os.getcwd()
    rospy.init_node('extract_data')
    scene_id = 43
    scene_id_path = "%06d" % scene_id
    scene_id_path = "/home/michael/data/scene_" + scene_id_path
    if not os.path.isdir(scene_id_path):
        os.makedirs(scene_id_path)
    extract_data(save_folder=scene_id_path, start_id=0)

if __name__ == "__main__":
    main()
