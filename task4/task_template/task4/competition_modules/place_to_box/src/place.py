#!/usr/bin/env python

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi,sqrt
from std_msgs.msg import String,Float64, Bool
from moveit_commander.conversions import pose_to_list
import rospy
import tf
from place_to_box.srv import *
import ik_4dof


class place_node(object):
	def __init__(self):
		self.listener = tf.TransformListener()
		self.node_name = "place_node"
		check = True
		while(check):
			check = False
			try:
				self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
			except:
				print "moveit server isn't open yet"
				check = True

		self.move_group_arm.set_goal_position_tolerance(0.05)
		self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

		self.place_srv = rospy.Service("place_to_box", tag, self.transform)
		self.place_srv = rospy.Service("home_place", home, self.home)
		self.place_srv = rospy.Service("close_grip", home, self.close)
		self.place_srv = rospy.Service("open_grip", home, self.open)
		self.grip_data = Float64()
	def transform(self, req):
		br = tf.TransformBroadcaster()
		tag = "tag_" + str(req.tag_id)
		try:
			now = rospy.Time.now()
			self.listener.waitForTransform('pi_camera', tag, now, rospy.Duration(3.0))
			(trans, rot) = self.listener.lookupTransform('pi_camera', tag , now)
			

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, \
			tf.Exception):
			rospy.loginfo("Tag not found!")
			return tagResponse("Tag not found!")

		pose_goal = geometry_msgs.msg.Pose()
	
		pose_goal.position.x = trans[2] - 0.21
		pose_goal.position.y = - trans[0]
		pose_goal.position.z = trans[1] + 0.02
		x1 = pose_goal.position.x

		print "Your box's position : " , pose_goal.position

		# if 0.15 <= pose_goal.position.x <= 0.21:
		# 	if -0.07 <= pose_goal.position.y <= 0.065:
		# 		self.special()
		# 		self._return()
		# 		return tagResponse("Process Successfully")

		# 	elif -0.15 <= pose_goal.position.y <= 0.165:
		# 		return tagResponse("Cannot arrive")

		for l in range(8):
			pose_goal.position.x = x1
			for i in range(4):
				degree = -90
				for j in range(90):
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

							self._return()
							return tagResponse("Process Successfully")

					degree += 1
				pose_goal.position.x -= 0.01
			pose_goal.position.z += 0.01

		return tagResponse("Cannot arrive objective pose!!")



	def home(self,req):
		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = -pi*5/13
		joint_goal[2] = pi*3/4
		joint_goal[3] = pi/3
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)
		return homeResponse("Home now!")		

	def special(self):
		joint_goal = self.move_group_arm.get_current_joint_values()
		joint_goal[0] = 0
		joint_goal[1] = pi*3/13
		joint_goal[2] = pi*1/4
		joint_goal[3] = pi/2
		joint_goal[4] = 0
		self.move_group_arm.go(joint_goal, wait=True)

	def _return(self):

		self.grip_data.data = 0.5
		self.pub_gripper.publish(self.grip_data)
		rospy.sleep(2)
		self.home(home)
		rospy.sleep(1)
		rospy.loginfo("End process")
	def open(self, req):
		self.grip_data.data = 0.5
		self.pub_gripper.publish(self.grip_data)
		rospy.loginfo("Open gripper")
		return homeResponse("Open gripper")	
	def close(self, req):
		self.grip_data.data = 1.5
		self.pub_gripper.publish(self.grip_data)
		rospy.loginfo("Close gripper")
		return homeResponse("Close gripper")	

	def onShutdown(self):
		rospy.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('place_node',anonymous=False)
	rospy.sleep(2)
	place_node = place_node()
	rospy.on_shutdown(place_node.onShutdown)
	rospy.spin()

