#!/usr/bin/env python2 

import rospy
import moveit_commander
import moveit_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_commander.conversions import pose_to_list
import copy
import sys

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
scale = 1.0
wpose = move_group.get_current_pose().pose
wpose.position.x = 0.3
# wpose.position.y = 0.0
wpose.position.z = 0.4
# waypoints = [copy.deepcopy(wpose)]
waypoints = [copy.deepcopy(wpose)]

# waypoints = []
# wpose = move_group.get_current_pose().pose
# # wpose.position.z -= scale * 0.1  # First move up (z)
# wpose.position.y += scale * 0.2  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.y -= scale * 0.1  # Third move sideways (y)
# waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

move_group.execute(plan, wait=True)