#!

import rospy
from std_msgs.msg import String

rospy.init_node('aiml_voice_recognizer')
pub=rospy.Publisher('voiceWords',String,queue_size=10)
r=rospy.Rate(1)

def get_voice(data):
    voice_text=data.data
    rospy.loginfo("I said:: %s",voice_text)
    pub.publish(voice_text)

def listener():
    rospy.loginfo("Starting voice recognizer")
    rospy.Subscriber("/recognizer/output",String,get_voice)
    rospy.spin()

while not rospy.is_shutdown():
    listener() 