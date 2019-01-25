#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_navigation.srv import robot_navigation
from math import sqrt
import time

class TagTracker(object):
    def __init__(self):
        # the tag to track
        self.tag_id_track = 1

        # the distance threshold between car and tag
        self.thres_distance = 0.05

        # the angle threshold between car and tag
        self.thres_angle = 0.1745 #10 degree

        # the flag for whether the robot has arrived the destination
        self.done_tracking = False

        # the lock for cbTag in case that calling service again before it returns
        self.cb_lock = False

        # setup service proxy
        rospy.wait_for_service('/robot_navigate')

        #setup publisher
        self.pub_cmd = rospy.Publisher('cmd_vel', TwistStamped, queue_size=1)

        
        self.Track()

    def Track(self):
        while done_tracking is not True:
            stat = rospy.wait_for_message("/odom", Odometry)
            theta = stat.twist.twist.angular.z
            if (abs(0-theta) > self.threshold): #rotate robot until it facing toward specified tag
                print "align"
                cmd = Twist()
                cmd.angular.z = theta/abs(theta) * 0.1
                self.pub_cmd.publish(cmd)
                continue
            else:
            	print "theta aligned"
            	cmd = Twist()
                cmd.angular.z = 0
                self.pub_cmd.publish(cmd)
            
            #now the robot should facing toward the tag
            try:
            	print "go pikachu"
                rospy.wait_for_service('/robot_navigate')
                ser_tag = rospy.ServiceProxy('/robot_navigate', robot_navigation)
                print (ser_tag(self.tag_id_track))
                time.sleep(10)
            except rospy.ServiceException as e:
                print e
                exit(1)

if __name__ == "__main__":
    rospy.init_node("tag_tracker",anonymous=False)
    joy_mapper = TagTracker()
    rospy.spin()
