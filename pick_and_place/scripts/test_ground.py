#!/usr/bin/env python2

import rospy
from pick_and_place.pick_and_place import PickAndPlace
from std_msgs.msg import Float64
from math import pi

if __name__ == "__main__":
    pick_and_place = PickAndPlace(0.05, 0.5)
    
    pick_and_place.setPickPose(0.4, 0.0, 0.1, 0, pi, 0)
    pick_and_place.setDropPose(0.0, 0.4, 0.4, 0, pi, 0)
    pick_and_place.setGripperPose(0.01, 0.01)
    
    pick_and_place.execute_pick_and_place()