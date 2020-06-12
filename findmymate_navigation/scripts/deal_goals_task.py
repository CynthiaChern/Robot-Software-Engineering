#!/usr/bin/env python
# -*- coding: UTF-8 -*-

""" nav_test.py - Version 1.1 2013-12-20

处理一系列发出来的goal
应该有一个最短更新间隔
定期扫描goal,选择最近的进行发送,但是上一个没有完成之前不会处理下一个
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class NavTest():
    def __init__(self):
        rospy.init_node('deal_goals', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        self.rate = rospy.Rate(1)
        ## 参数定义
        self.get_new_goal = False        
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=5)
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        #--------------------------------链接move_base 服务------------------------------
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        #-------------------------------设置初始位姿--------------------------------------
        #-------------------------------阻塞进程----------------------------------------
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        self.initial_pose = PoseWithCovarianceStamped()        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        # Make sure we have the initial pose
        while self.initial_pose.header.stamp == "":
            rospy.sleep(1)
        #-------------------------设置初始变量------------------------------------------------
        # Variables to keep track of success rate, running time,
        # and distance traveled
        i = 0
        n_goals = 0
        n_successes = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        #---------------------等待goal唤醒---------------------------------
        self.last_location = Pose()        
        self.location = Pose()        
        rospy.wait_for_message("navigation_goal",Pose)
        self.goal_sub = rospy.Subscriber("navigation_goal",Pose,self.update_goal,queue_size=1)        
        #----------------------开始导航----------------------------------------
        rospy.loginfo("Starting navigation test")
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            if not self.get_new_goal:
                rospy.loginfo("wait for new goal.")
                self.rate.sleep()                            
                continue
            self.get_new_goal = False
            # 更新初始化位姿,实时更新位姿
            if self.initial_pose.header.stamp == "":
                distance = sqrt(pow(self.location.position.x - 
                                    self.last_location.position.x, 2) +
                                pow(self.location.position.y - 
                                    self.last_location.position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(self.location.position.x - 
                                    self.last_location.position.x, 2) +
                                pow(self.location.position.y - 
                                    self.last_location.position.y, 2))
                self.initial_pose.header.stamp = ""
            # 存储上一个地点
            self.last_location = self.location
            # 增加计时器
            i += 1
            n_goals += 1
            # 跟新目标
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = self.location
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            # 终端输出
            rospy.loginfo("Going to: %f %f %f"%(self.location.position.x,self.location.position.y,self.location.position.z))
            # 开始机器人向下一个节点            
            self.move_base.send_goal(self.goal,done_cb= self.donecb,active_cb=self.activecb,feedback_cb=self.feedbackcb)
            # 容限时间
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            # 检查是否成果
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            self.rate.sleep()
    def donecb(self,state,result):
        return 
    def activecb(self):
        return 
    def feedbackcb(self,fb):
        if self.get_new_goal :
            self.move_base.cancel_goal()
            rospy.loginfo("cancel_goal and send new goal")
        else:
            return
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
    def update_goal(self,target_location):
        self.location = target_location
        self.get_new_goal = True
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
