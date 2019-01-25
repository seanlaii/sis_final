#!/usr/bin/env python

from pose_estimate_and_pick.srv import *
from object_detection.srv import *
import rospy
import geometry_msgs.msg
from math import pi,sqrt
from std_msgs.msg import String,Float64, Bool

class client_task2(object):
	def __init__(self):

		rospy.wait_for_service('pick_home')
		self.pick_service = rospy.ServiceProxy('/pick_home', home)
		print("Pick_home service already!!")

		self.task2 = rospy.Service('task2', task2_srv, self.task2_ser)
		rospy.wait_for_service('pose_estimation')
		self.pose_service = rospy.ServiceProxy('/pose_estimation', pose_estimation)
		print("Pose_estimation service already!!")

		rospy.wait_for_service('pick')
		self.pick_service = rospy.ServiceProxy('/pick', pick_srv)
		print("Picking service already!!")


	def task2_ser(self, req):
		pick_home_res = self.pick_service()
		pose_msg = self.pose_service()
		for i in range(len(pose_msg.obj_list)):
			result = self.pick_service(pose_msg.pose[i])
			if result.result:
				return task2_srvResponse(pose_msg.tagID[i])
		return task2_srvResponse("Task 2 is Fail")

	def onShutdown(self):
		rospy.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('client_task2',anonymous=False)
	rospy.sleep(2)
	client_task2 = client_task2()
	rospy.on_shutdown(client_task2.onShutdown)
	rospy.spin()