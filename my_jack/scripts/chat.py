#!/usr/bin/env python



import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys
from subprocess import call
from geometry_msgs.msg import Twist
from math import radians
import os
from turtlebot_msgs.srv import SetFollowState


class Chat:
    def __init__(self, script_path):
        rospy.init_node('chat')

        rospy.on_shutdown(self.cleanup)
        
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
        
        self.soundhandle = SoundClient(blocking=True)
        
        rospy.sleep(1)
        
        self.soundhandle.stopAll()
        
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
       
        rospy.loginfo("Ready, waiting for commands...")
	self.soundhandle.say('Hello,What can I do for you?')
	
        rospy.Subscriber('/lm_data', String, self.talkback)


    def talkback(self, msg):
       
        rospy.loginfo(msg.data)

	if msg.data.find('DAY')>-1:
        	rospy.loginfo("It is really graet.I had a nice day with Cynthia.")
                self.soundhandle.say("It is really graet.I had a nice day with Cynthia.", volume=0.1)	 
	elif msg.data.find('FROM')>-1:
        	rospy.loginfo("I am from your heart.")
                self.soundhandle.say("I am from your heart.", volume=0.1)   	
	elif msg.data.find('NAME')>-1:
        	rospy.loginfo("I am Jack.Nice to meet you.")
        	self.soundhandle.say("I am Jack.Nice to meet you.", volume=0.1)
        elif msg.data.find('PHOTO')>-1:
        	rospy.loginfo("I am delighted to do this for you.")
        	self.soundhandle.say("I am delighted to do this for you.", volume=0.1)	
	elif msg.data.find('OLD')>-1:
        	rospy.loginfo("It's a secret,but I am really young.")
        	self.soundhandle.say("It's a secret,but I am really young.", volume=0.1)	
        elif msg.data.find('CYNTHIA')>-1:
        	rospy.loginfo("She is my best friend.")
        	self.soundhandle.say("She is my best friend.", volume=0.1)
	elif msg.data.find('BYE')>-1:
        	rospy.loginfo("see you")
        	self.soundhandle.say("see you", volume=0.1)
        elif msg.data.find('JACK')>-1:
        	rospy.loginfo("Hello, my friend. Let's have a chat.")
        	self.soundhandle.say("Hello, my friend. Let's have a chat.", volume=0.1)
        elif msg.data.find('INTRODUCE')>-1:
                rospy.loginfo("I am your friend,Jack. I can do lots of things")
                self.soundhandle.say("I am Jack.Nice to meet you.", volume=0.1)

		

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down chat node...")

if __name__=="__main__":
    try:
        Chat(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("chat node terminated.")
