#!/usr/bin/env python

import rospy, os, sys
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from opencv_apps.msg import FaceArrayStamped
from opencv_apps.msg import RotatedRectStamped
from opencv_apps.msg import Face
from opencv_apps.msg import Rect
from opencv_apps.msg import RotatedRect
from opencv_apps.msg import Point2D


class MyBot:
     def __init__(self, script_path):
         rospy.init_node('mybot')

         rospy.on_shutdown(self.cleanup)

         # Create the sound client object
         #self.soundhandle = SoundClient()
         self.soundhandle = SoundClient(blocking=True)

         # Wait a moment to let the client connect to the sound_play server
         rospy.sleep(1)

         # Make sure any lingering sound_play processes are stopped.
         self.soundhandle.stopAll()

         
         #Announce that we are ready
        
         rospy.loginfo("Say one of the navigation commands...")

         # Subscribe to the recognizer output and set the callback function
         rospy.Subscriber('/lm_data', String, self.talkback)

         #the position of face detected
         self.face_x=0
         self.face_y=0
         # Subscribe to the face_detection output
         rospy.Subscriber('/face_detection/faces', FaceArrayStamped, self.face_back)
  
         # the center of blue object
         self.blue_x = 0
         self.blue_width = 0
         # Subscribe to the blue_tracting output
         rospy.Subscriber('/camshift/track_box', RotatedRectStamped, self.blue_back)

         #Publish to the take_photo topic to use take_photo node
         self.take_photo = rospy.Publisher("/take_photo", String, queue_size=10)

     def blue_back(self,blue_data):
          self.blue_x = blue_data.rect.center.x 
          self.blue_width = blue_data.rect.size.width
             
     def face_back(self,face_data):
         pos = face_data.faces
         if pos:
             self.face_x=pos[0].face.x
             self.face_y=pos[0].face.y

     def talkback(self, msg):
         #Print the recognized words on the screen
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
         elif msg.data.find('BYE')>-1:
        	rospy.loginfo("see you")
        	self.soundhandle.say("see you", volume=0.1)
         elif msg.data.find('PHOTO')>-1:
                rospy.loginfo("I am delighted to do this for you.")
                self.soundhandle.say("I am delighted to do this for you.", volume=0.1)
                i=0
                while(i<1):
                 if self.face_x<240 and self.face_x>0:
                         rospy.loginfo("Please stand a little to the left.")
                         self.soundhandle.say("Please stand a little to the left.", volume=0.1)
                         self.face_x=0 
                         self.face_y=0
                         rospy.sleep(1)
                 elif self.face_x>420:
                         rospy.loginfo("Please stand a little to the right.")
                         self.soundhandle.say("Please stand a little to the right.", volume=0.1)
                         self.face_x=0
                         self.face_y=0
                         rospy.sleep(1)
                 elif self.face_x>=240 and self.face_x<=420:
                         rospy.loginfo("OK, that is good.")
                         self.soundhandle.say("OK, thst is good.", volume=0.1)
                         break
                 elif self.face_x==0 or self.face_y==0:
                         rospy.loginfo("I can't catch your face, please stand and face to me.")
                         self.soundhandle.say("I can't catch your face, please stand and face to me.", volume=0.1)
                         rospy.loginfo("3! 2! 1! Cheese!")
                         self.soundhandle.say("3! 2! 1! Cheese", volume=0.1)
                         self.take_photo.publish('take photo')
                         rospy.loginfo("You have a photo now.")
                         self.soundhandle.say("You have a photo now.", volume=0.1)
                         i=i+1
         elif msg.data=='':
             rospy.sleep(1)
         else:
             rospy.loginfo("TSorry I can not understand, can you repeat it?")
             self.soundhandle.say("Sorry I can not understand, can you repeat it?", volume=0.1)
             #rospy.sleep(2)

     def cleanup(self):
         self.soundhandle.stopAll()
         rospy.loginfo("Shutting down talkbot node...")

if __name__=="__main__":
    try:
        MyBot(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mybot node terminated.")
