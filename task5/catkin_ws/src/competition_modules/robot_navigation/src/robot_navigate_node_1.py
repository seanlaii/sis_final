#!/usr/bin/env python

import rospy
import tf
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_navigation.srv import robot_navigation
from apriltags2_ros.msg import AprilTagDetection
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
import numpy as np
class RobotNavigate(object):
    def __init__(self):
        self.srv_navigate = rospy.Service("/robot_navigate", robot_navigation, self.cbNavigate)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.thres_angle = 0.1745 #10 degree
        # database of absolute pose of each tag
        self.park_x = {'0':2.47, '1':2.47, '2':2.47, '5':5.75}
        self.park_y = {'0':0.90, '1':0.90, '2':0.90, '5':5.91}
        self.park_th = {'0':0, '1':0, '2':0, '5':pi}

        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
    def cbNavigate(self, req):      
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        
        if park_x.get(str(req.id)) is None:
            return "NotSupportedTag"

        theta_goal = park_x[str(req.id)]
        stat = rospy.wait_for_message("/odom", Odometry)
        theta = stat.twist.twist.angular.z
        while abs(theta_goal - theta) > self.threshold:
            cmd = Twist()
            cmd.angular.z = theta/abs(theta) * 0.1
            self.pub_cmd.publish(cmd)
            stat = rospy.wait_for_message("/odom", Odometry)
            theta = stat.twist.twist.angular.z
                    
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.park_x[str(req.id)]
        goal.target_pose.pose.position.y = self.park_y[str(req.id)]
        goal.target_pose.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, self.park_th[str(req.id)])

        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        # Go! Pikachu!
        self.client.send_goal(goal)
        _result = self.client.wait_for_result(rospy.Duration(10)) 
        if _result == False:
            self.client.cancel_goal()
        return str(_result)

if __name__ == '__main__':
    rospy.init_node('robot_navigation',anonymous=False)
    node = RobotNavigate()
    rospy.spin()
