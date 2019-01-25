#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi,sqrt
from std_msgs.msg import String,Float64, Bool
from moveit_commander.conversions import pose_to_list
import rospy
import tf
from pose_estimate_and_pick.srv import *
import ik_4dof




class pick_node(object):
	def __init__(self):
		self.listener = tf.TransformListener()
		self.node_name = "pick_node"
		check = True
		while(check):
			check = False
			try:
				self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
			except:
				print "moveit server isn't open yet"
				check = True

		self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

		self.place_srv = rospy.Service("pick", pick_srv, self.pick_cb)
		self.place_srv = rospy.Service("go_home",home, self.home)

	def pick_cb(self,req):
		br = tf.TransformBroadcaster()
		tf_name = req.tf
		print(tf_name)

		pose_goal = geometry_msgs.msg.Pose()
	
		pose_goal.position.x = tf_name.position.x 
		pose_goal.position.y = tf_name.position.y 
		pose_goal.position.z = tf_name.position.z - 0.005


		print "Your object's position : " , pose_goal.position

		degree = -90
		for j in range(20):
			joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, degree)

			if len(joint_value) > 0:

				for joint in joint_value:
					joint = list(joint)
					# determine gripper state
					joint.append(0)
					try:
						self.move_group_arm.go(joint, wait=True)
						self.move_group_arm.stop()
					except:
						rospy.loginfo(str(joint) + " isn't a valid configuration.")
						continue
					
					grip_data = Float64()
					grip_data.data = 1.5
					self.pub_gripper.publish(grip_data)
					rospy.sleep(2)


					self.home_4()
					rospy.sleep(2)
					grip_data.data = 0.0
					self.pub_gripper.publish(grip_data)

					
					rospy.loginfo("End process")


					return pick_srvResponse(True)
			degree += 1
		return pick_srvResponse(False)		

	def home(self,req):
		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi*5/13
		joint_goal[2] = pi*3/7
		joint_goal[3] = pi/9
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)
		return homeResponse("Home now!")		

	def onShutdown(self):
		rospy.loginfo("Shutdown.")

	def home_4(self):
		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi*5/13
		joint_goal[2] = pi*3/4
		joint_goal[3] = pi/3
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)

if __name__ == '__main__': 
	rospy.init_node('pick_node',anonymous=False)
	rospy.sleep(2)
	pick_node = pick_node()
	rospy.on_shutdown(pick_node.onShutdown)
	rospy.spin()
