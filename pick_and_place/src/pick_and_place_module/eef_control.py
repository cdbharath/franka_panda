#!/usr/bin/env python2 

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list
from copy import deepcopy
from math import pi, fabs, sqrt, cos

tau = 2*pi
def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))



class MoveGroupControl:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_control", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        planning_frame = move_group.get_planning_frame()
        # rospy.loginfo(" Planning frame: %s", planning_frame)

        eef_link = move_group.get_end_effector_link()
        # rospy.loginfo(" End effector link: %s", eef_link)

        group_names = robot.get_group_names()
        # rospy.loginfo(" Available Planning Groups: %s", robot.get_group_names())

        # rospy.loginfo(" Printing robot state:")
        # rospy.loginfo(robot.get_current_state())

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, j1=0, j2=0, j3=0, j4=-tau/4, j5=0, j6=0, j7=0):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        rospy.loginfo("Joint goal: %s", joint_goal)
        joint_goal[0] = j1
        joint_goal[1] = j2
        joint_goal[2] = j3
        joint_goal[3] = j4
        joint_goal[4] = j5
        joint_goal[5] = j6
        joint_goal[6] = j7
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_pose_goal(self, x=0, y=0, z=0, X=0, Y=0, Z=0):
        move_group = self.move_group

        qw, qx, qy, qz = quaternion_from_euler(X, Y, Z)
        
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = qw
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def follow_cartesian_path(self, waypoints):
        move_group = self.move_group
        pose = self.get_current_pose()
        cartesian_points = []
        
        for waypoint in waypoints:
            qw, qx, qy, qz = quaternion_from_euler(waypoint[3], waypoint[4], waypoint[5])
            
            pose_ = deepcopy(pose)
            pose_.position.x = waypoint[0]
            pose_.position.y = waypoint[1]
            pose_.position.z = waypoint[2]
            pose_.orientation.w = qw
            pose_.orientation.x = qx
            pose_.orientation.y = qy
            pose_.orientation.z = qz        
            
            cartesian_points.append(pose_)
            
            (plan, _) = move_group.compute_cartesian_path(
                                   cartesian_points,    # waypoints to follow
                                   0.01,                # eef_step
                                   0.0)                 # jump_threshold
            move_group.execute(plan, wait=True)

    def get_current_joint_states(self):
        move_group = self.move_group
        joint_states = move_group.get_current_joint_values()
        return joint_states
    
    def get_current_robot_states(self):
        robot = self.robot
        current_states = robot.get_current_state()
        return current_states
    
    def get_current_pose(self):
        move_group = self.move_group
        pose = move_group.get_current_pose().pose
        return pose
    
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


# if __name__ == "__main__":

#     moveit_control = MoveGroupControl()
#     # moveit_control.go_to_joint_state()    
#     moveit_control.go_to_pose_goal(0.5, 0.0, 0.5, 0, 0, 0)
#     # joint_states = moveit_control.get_current_joint_states()
#     # moveit_control.go_to_joint_state(joint_states[0], joint_states[1], joint_states[2], joint_states[3], joint_states[4], joint_states[5], joint_states[6])    

#     # moveit_control.go_to_joint_state(j4=-pi/4-pi/8, j6=pi/4 + pi/8, j7=pi/4)    
#     # moveit_control.go_to_joint_state(j4=-pi/4-pi/8-pi/20, j6=pi/4 + pi/8 + pi/20, j7=pi/4)    

#     # moveit_control.go_to_pose_goal(0.5, 0.0, 0.9, pi/4, 0, pi)
#     # moveit_control.go_to_pose_goal(0.5, 0.0, 0.8, pi/4, 0, pi)
