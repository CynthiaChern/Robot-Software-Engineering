#!

import rospy
import aiml
import os
import sys
from std_msgs.msg import String

rospy.init_node('aiml_voice_server')
mybot = aiml.Kernel()
response_publisher = rospy.Publisher('response',String,queue_size=10)

def load_aiml(xml_file):
    data_path = rospy.get_param("aiml_path")
    print data_path
    os.chdir(data_path)
    if os.path.isfile("standard.brn"):
        mybot.bootstrap(brainFile="standard.brn")
    else:
        mybot.bootstrap(learnFiles=xml_file,commands="load aiml b")
        mybot.saveBrain("standard.brn")

def callback(data):
    input = data.data
    response=mybot.respond(input)

    rospy.loginfo("I heard:: %S",data.data)
    rospy.loginfo("I spoke:: %s",response)
    response_publisher.publish(response)

def listener():
    rospy.loginfo("starting ros aiml voice server")
    rospy.Subscriber("voiceWords",String,callback )
    rospy.spin()

if _name_ == '_main_':
    load_aiml('startup.xml')
    listener()