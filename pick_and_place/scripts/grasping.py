#!/usr/bin/env python2

import rospy
from pick_and_place.eef_control import MoveGroupControl
from std_msgs.msg import Float64
from math import pi
import tf

if __name__ == "__main__":
    # rospy.init_node("grasping")
    moveit_control = MoveGroupControl()

    finger1_pub = rospy.Publisher('/panda_finger1_controller/command', Float64, queue_size=10)
    finger2_pub = rospy.Publisher('/panda_finger2_controller/command', Float64, queue_size=10)
    rospy.sleep(1)
        
    finger1_data = Float64()
    finger1_data.data = 0.05
    finger2_data = Float64()
    finger2_data.data = 0.05
    
    # moveit_control.go_to_joint_state()    
    # moveit_control.go_to_pose_goal(0.5, 0.0, 0.5, 0, 0, 0)
    # joint_states = moveit_control.get_current_joint_states()
    # moveit_control.go_to_joint_state(joint_states[0], joint_states[1], joint_states[2], joint_states[3], joint_states[4], joint_states[5], joint_states[6])    

    # moveit_control.go_to_joint_state(j4=-pi/4-pi/8, j6=pi/4 + pi/8, j7=pi/4)    
    # moveit_control.go_to_joint_state(j4=-pi/4-pi/8-pi/20, j6=pi/4 + pi/8 + pi/20, j7=pi/4)    

    # moveit_control.go_to_pose_goal(0.5, 0.0, 0.9, pi/4, 0, pi)
    
    # finger1_pub.publish(finger1_data)
    # finger2_pub.publish(finger2_data)
    pose = moveit_control.get_pose()
    (x, y, z) = (pose.position.x, pose.position.y, pose.position.z) 
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
    print(roll, pitch, yaw)    
    moveit_control.go_to_pose_goal(0.5, 0.0, 0.7, -pi/4, 0, pi) # yaw - pi/4, _, roll
    
    rospy.loginfo("Ending gracefully")
    