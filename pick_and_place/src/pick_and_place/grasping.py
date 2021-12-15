#!/usr/bin/env python2 

import rospy
from std_msgs.msg import Float64

class Gripper:
    def __init__(self):
        self.finger1_pub = rospy.Publisher('/panda_finger1_controller/command', Float64, queue_size=10)
        self.finger2_pub = rospy.Publisher('/panda_finger2_controller/command', Float64, queue_size=10)
        rospy.sleep(1)
        
    def grasp(self, finger1_y, finger2_y):
        finger1_data = Float64()
        finger1_data.data = finger1_y
        finger2_data = Float64()
        finger2_data.data = finger2_y
        
        self.finger1_pub.publish(finger1_data)
        self.finger2_pub.publish(finger2_data)
        
        
