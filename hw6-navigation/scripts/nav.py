#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import actionlib

from std_msgs.msg import String
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

original = 0
start = 1

class NavToPoint:
    def __init__(self):
        rospy.init_node('nav_core')

	rospy.Subscriber('/nav', String, self.goto)
        
        rospy.on_shutdown(self.cleanup)
        
	# Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #publish when reach destination
        #self.navigation = rospy.Publisher("/nav_find_point", String, queue_size=10)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")
            
        rospy.loginfo("Ready")

	rospy.sleep(1)

	self.locations = dict()
	# Location A(sofa)
	A_x = -3
	A_y = 3.19
	A_theta = 0
	quaternionA = quaternion_from_euler(0.0, 0.0, A_theta)
	self.locations['A'] = Pose(Point(A_x, A_y, 0.000), Quaternion(quaternionA[0], quaternionA[1], quaternionA[2], quaternionA[3]))

        # Location B(livingroom)
	B_x = 1.61
	B_y = 3.27
	B_theta = 0
	quaternionB = quaternion_from_euler(0.0, 0.0, B_theta)
	self.locations['B'] = Pose(Point(B_x, B_y, 0.000), Quaternion(quaternionB[0], quaternionB[1], quaternionB[2], quaternionB[3]))

	# Location C(kitchen)
	C_x = -1.34
	C_y = -2.1
	C_theta = 0
	quaternionC = quaternion_from_euler(0.0, 0.0, C_theta)
	self.locations['C'] = Pose(Point(C_x, C_y, 0.000), Quaternion(quaternionC[0], quaternionC[1], quaternionC[2], quaternionC[3]))


	self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")


    def goto(self,data):
		
	self.goal.target_pose.header.frame_id = 'map'
	self.goal.target_pose.header.stamp = rospy.Time.now()

	# Robot will go to the bookshelf
	if data.data == 'sofa':
	    rospy.loginfo("Going to the sofa")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['A']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
            if waiting == 1:
		rospy.loginfo("Reached the sofa")
		#self.navigation.publish('Reached the sofa')
        # Robot will go to the bedroom
	elif data.data=='livingroom':
	    rospy.loginfo("Going to the livingroom")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['B']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
	    if waiting == 1:
	        rospy.loginfo("Reached the livingroom")
		#self.navigation.publish('reached the livingroom')
        # Robot will go to the kitchen
	elif data.data == 'kitchen':
	    rospy.loginfo("Going to the kitchen")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['C']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
	    if waiting == 1:
		rospy.loginfo("Reached the kitchen")
		#self.navigation.publish('reached the kitchen')
		# Robot will go to the table
	

    def cleanup(self):
        rospy.loginfo("Shutting down navigation	....")
	self.move_base.cancel_goal()

if __name__=="__main__":
    
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass


