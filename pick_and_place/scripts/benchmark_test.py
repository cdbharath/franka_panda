#!/usr/bin/env python2

import rospy
from pick_and_place_module.pick_and_place import PickAndPlace
from pick_and_place_module.eef_control import MoveGroupControl
from std_msgs.msg import Float64
from math import pi
import tf

def pick():
    pick_and_place = PickAndPlace(0.05, 0.5)
    
    pick_and_place.setDropPose(0.0, 0.4, 0.4, 0, pi, 0)
    pick_and_place.setPickPose(0.51, 0.02, 0.08, 0*3.14/180, pi, 0)
    pick_and_place.setGripperPose(0.01, 0.01)
    
    # pick_and_place.execute_pick_and_place()
    # pick_and_place.execute_cartesian_pick_and_place()
    pick_and_place.execute_cartesian_pick_up()


def test_benchmark():
    moveit_control = MoveGroupControl()
    # pick_and_place = PickAndPlace(0.05, 0.5)
    
    # pick_and_place.setDropPose(0.0, 0.4, 0.4, 0, pi, 0)
    # pick_and_place.setPickPose(0.50, 0.0, 0.12, 0*3.14/180, pi, 0)
    # pick_and_place.setGripperPose(0.008, 0.008)
    
    
    # Pick up the object
    # pick_and_place.execute_cartesian_pick_up()

    # Rotate the object
    pose = moveit_control.get_current_pose()
    (x, y, z) = (pose.position.x, pose.position.y, pose.position.z) 
    
    # TODO check roll, pitch yaw order
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z))
        
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll + pi/4, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll - pi/4, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

    roll = roll - pi/4
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch + pi/4, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch - pi/4, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])

    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw + pi/4]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw - pi/4]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])
    roll = roll + pi/4
    
    # Shake the object
    moveit_control.follow_cartesian_path([[x, y, z + 0.1, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z - 0.1, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z + 0.1, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z - 0.1, roll, pitch, yaw]])
    moveit_control.follow_cartesian_path([[x, y, z, roll, pitch, yaw]])


if __name__ == "__main__":
    pick()
    test_benchmark()