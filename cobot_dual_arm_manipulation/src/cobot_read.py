#!/usr/bin/env python


import numpy as np
import rospy
import math
import tf
import sys
import copy
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import random
# import baxter_interface
import cv2
import cv_bridge
import rospkg
import os

from scipy.optimize import minimize
from math import sqrt
from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix
from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from moveit_commander import MoveGroupCommander
# from baxter_core_msgs.msg import (DigitalIOState, EndEffectorState)
# from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image


# Initialize moveit commander and move group commander
def InitializeMoveitCommander():

	#First initialize moveit_commander 
	print "============ Starting tutorial setup"
	joint_state_topic = ['/joint_states']
	moveit_commander.roscpp_initialize(joint_state_topic)

	#Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
	robot = moveit_commander.RobotCommander()

	#Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
	global group_both_arms, group_left_arm, group_right_arm
	group_both_arms = MoveGroupCommander("dual_arm")
	group_both_arms.set_goal_position_tolerance(0.05)
	group_both_arms.set_goal_orientation_tolerance(0.05)
	group_both_arms.set_planning_time(5.0)

	group_left_arm = MoveGroupCommander("indy7")
	group_left_arm.set_goal_position_tolerance(0.05)
	group_left_arm.set_goal_orientation_tolerance(0.05)
	group_left_arm.set_planning_time(5.0)

	group_right_arm = MoveGroupCommander("irb120")
	group_right_arm.set_goal_position_tolerance(0.05)
	group_right_arm.set_goal_orientation_tolerance(0.05)
	group_right_arm.set_planning_time(5.0)

	#We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	# Obtain current poses of left and right end-effectors: [x,y,z,roll,pitch,yaw].T
	P_left_pose = group_left_arm.get_current_pose()
	P_right_pose = group_right_arm.get_current_pose()
	P_left_euler = group_left_arm.get_current_rpy()
	P_right_euler = group_right_arm.get_current_rpy()

	global left_angles, right_angles
	left_angles = group_left_arm.get_current_joint_values()
	right_angles = group_right_arm.get_current_joint_values()

	global P_left_current, P_right_current
	P_left_current = np.array([[P_left_pose.pose.position.x],[P_left_pose.pose.position.y],[P_left_pose.pose.position.z],[P_left_euler[0]],[P_left_euler[1]],[P_left_euler[2]]])
	P_right_current = np.array([[P_right_pose.pose.position.x],[P_right_pose.pose.position.y],[P_right_pose.pose.position.z],[P_right_euler[0]],[P_right_euler[1]],[P_right_euler[1]]])

	print "angles current left:"
	print left_angles
	print "angles current right:"
	print right_angles
	print "xyz rpw current left indy:"
	print P_left_current
	print "xyz rpw current right ibb:"
	print P_right_current


	print group_left_arm.get_current_pose().pose
	print group_right_arm.get_current_pose().pose

	#P_left_current = np.array([[0.18],[0.64],[-0.16],[-0.5],[0.8],[0.005]])
	#P_right_current = np.array([[0.56],[-0.66],[0.51],[-0.31],[0.83],[-0.29]])
	


# Main portion of code
def main():

	# Initialize node
	rospy.init_node('read_goal_position')

	# Start Moveit Commander
	InitializeMoveitCommander()


if __name__ == '__main__':
	main()
