#!/usr/bin/env python2 

import rospy
from eef_control import MoveGroupControl
from grasping import Gripper
from copy import deepcopy
from math import pi

class PickAndPlace:
    def __init__(self, gripper_offset, intermediate_z_stop):
        self.gripper_offset = gripper_offset
        self.intermediate_z_stop = intermediate_z_stop
        self.pick_pose = None
        self.place_pose = None
        self.gripper_pose = None
        self.moveit_control = MoveGroupControl()
        self.gripper = Gripper()
    
    def setPickPose(self, x, y, z, roll, pitch, yaw):
        self.pick_pose = [x, y, z, roll + pi/4, pitch, yaw]
    
    def setDropPose(self, x, y, z, roll, pitch, yaw):
        self.drop_pose = [x, y, z, roll + pi/4, pitch, yaw]
    
    def setGripperPose(self, finger1, finger2):
        self.gripper_pose = [finger1, finger2]
    
    def generate_waypoints(self, destination_pose, action):
        '''
        Generated waypoints are for a particular application
        This is to be changed based on the application it is being used
        '''
        move_group = self.moveit_control

        waypoints = []

        if action:
            current_pose = move_group.get_current_pose()
            current_pose_ = deepcopy(destination_pose)
            current_pose_[0] = current_pose.position.x
            current_pose_[1] = current_pose.position.y
            current_pose_[2] = self.intermediate_z_stop
            waypoints.append(current_pose_)
        
        intermediate_pose = deepcopy(destination_pose)
        intermediate_pose[2] = self.intermediate_z_stop
        waypoints.append(intermediate_pose)
        if not action:
            destination_pose_ = deepcopy(destination_pose)
            destination_pose_[2] = destination_pose_[2]  + 0.1 
            waypoints.append(destination_pose_)

            destination_pose_ = deepcopy(destination_pose)
            destination_pose_[2] = destination_pose_[2]  + self.gripper_offset 
            waypoints.append(destination_pose_)
        
        return waypoints
    
    def execute_cartesian_pick_and_place(self):

        waypoints = self.generate_waypoints(self.pick_pose, 0)
        rospy.loginfo("Generated waypoints for pick: %s", waypoints)
        self.moveit_control.follow_cartesian_path(waypoints)

        self.gripper.grasp(self.gripper_pose[0], self.gripper_pose[1])
        rospy.sleep(2)
    
        waypoints = self.generate_waypoints(self.drop_pose, 1)
        rospy.loginfo("Generated waypoints for drop: %s", waypoints)
        self.moveit_control.follow_cartesian_path(waypoints)

        self.gripper.grasp(0.05, 0.05)

    def execute_pick_and_place(self):
        move_group = self.moveit_control

        self.gripper.grasp(0.05, 0.05)
        rospy.sleep(2)        

        waypoints = self.generate_waypoints(self.pick_pose, 0)
        
        for waypoint in waypoints:
            rospy.loginfo("Executing waypoint: %s", waypoint)
            move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])

        self.gripper.grasp(self.gripper_pose[0], self.gripper_pose[1])
        rospy.sleep(2)
            
        waypoints = self.generate_waypoints(self.drop_pose, 1)
        
        for waypoint in waypoints:
            rospy.loginfo("Executing waypoint: %s", waypoint)
            move_group.go_to_pose_goal(waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5])
                        
        self.gripper.grasp(0.05, 0.05)
        rospy.sleep(2)        
