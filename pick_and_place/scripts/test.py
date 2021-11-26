#!/usr/bin/env python

from pick_and_place.eef_control import MoveGroupControl

if __name__ == "__main__":
    moveit_control = MoveGroupControl()
    # moveit_control.go_to_joint_state()    
    moveit_control.go_to_pose_goal(0.5, 0.0, 0.5, 0, 0, 0)
    # joint_states = moveit_control.get_current_joint_states()
    # moveit_control.go_to_joint_state(joint_states[0], joint_states[1], joint_states[2], joint_states[3], joint_states[4], joint_states[5], joint_states[6])    

    # moveit_control.go_to_joint_state(j4=-pi/4-pi/8, j6=pi/4 + pi/8, j7=pi/4)    
    # moveit_control.go_to_joint_state(j4=-pi/4-pi/8-pi/20, j6=pi/4 + pi/8 + pi/20, j7=pi/4)    

    # moveit_control.go_to_pose_goal(0.5, 0.0, 0.9, pi/4, 0, pi)
    # moveit_control.go_to_pose_goal(0.5, 0.0, 0.8, pi/4, 0, pi)
