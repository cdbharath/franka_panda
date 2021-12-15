#!/usr/bin/env python2 

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController



if __name__ == "__main__":
    rospy.init_node("test_ground")
    
    # pub = rospy.Publisher("/panda_hand_controller/command", JointTrajectory, queue_size=10)
    
    # hand_msg = JointTrajectory()
    # hand_msg.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
    
    # point_msg = JointTrajectoryPoint()
    # point_msg.positions = [0.0, 0.0]
    # hand_msg.points = point_msg 

    # pub.publish(hand_msg)
    
    # pub = rospy.Publisher("/panda_arm_controller/command", JointTrajectory, queue_size=10)
    
    pub = rospy.Publisher("/joint_position_controller/command", JointTrajectory, queue_size=10)
    # rospy.wait_for_service('/controller_manager/switch_controller')
    # try:
    #     sc_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    #     start_controllers = ['joint_position_controller']
    #     stop_controllers = ['panda_hand_controller','panda_arm_controller']
    #     strictness = 2
    #     start_asap = False
    #     timeout = 0.0
    #     res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
    #     rospy.loginfo('switching successful')
    # except rospy.ServiceException as e:
    #     rospy.loginfo("Service Call Failed")	

    
    arm_msg = JointTrajectory()
    arm_msg.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
    
    point_msg = JointTrajectoryPoint()
    point_msg.positions = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    arm_msg.points = point_msg
    
    pub.publish(arm_msg)
