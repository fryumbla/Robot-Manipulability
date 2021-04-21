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

from sensor_msgs.msg import Image


global first_flag, force_left_gripper, force_right_gripper
first_flag = False
force_left_gripper = False
force_right_gripper = False


# Initialize moveit commander and move group commander
def InitializeMoveitCommander():

	#First initialize moveit_commander 
	print "============ Starting tutorial setup"
	# joint_state_topic = ['indy7/joint_states']
	# moveit_commander.roscpp_initialize(joint_state_topic)

	#Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
	robot1 = moveit_commander.RobotCommander()
	# robot2 = moveit_commander.RobotCommander(robot_description="/indy7/robot_description")

	# We can get a list of all the groups in the robot:
	print("============ Available Planning Groups:", robot1.get_group_names())
	# print("============ Available Planning Groups:", robot2.get_group_names())	
	# Sometimes for debugging it is useful to print the entire state of the
	# robot:
	print("============ Printing robot state")
	# print(robot1.get_current_state())
	# print(robot2.get_current_state())
	print("")

	#Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
	# global scene1, scene2
	# scene1 = moveit_commander.PlanningSceneInterface(robot_description="/irb120/robot_description")
	# scene2 = moveit_commander.PlanningSceneInterface(robot_description="/indy7/robot_description")
	# rospy.sleep(1)

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
	group_right_arm.set_goal_position_tolerance(0.001)
	group_right_arm.set_goal_orientation_tolerance(0.001)
	group_right_arm.set_planning_time(5.0)

	#We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
	# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	# Obtain current poses of left and right end-effectors: [x,y,z,roll,pitch,yaw].T
	# P_left_pose = group_left_arm.get_current_pose()
	# P_right_pose = group_right_arm.get_current_pose()
	# P_left_euler = group_left_arm.get_current_rpy()
	# P_right_euler = group_right_arm.get_current_rpy()

	# global P_left_current, P_right_current
	# P_left_current = np.array([[P_left_pose.pose.position.x],[P_left_pose.pose.position.y],[P_left_pose.pose.position.z],[P_left_euler[0]],[P_left_euler[1]],[P_left_euler[2]]])
	# P_right_current = np.array([[P_right_pose.pose.position.x],[P_right_pose.pose.position.y],[P_right_pose.pose.position.z],[P_right_euler[0]],[P_right_euler[1]],[P_right_euler[1]]])

	#P_left_current = np.array([[0.18],[0.64],[-0.16],[-0.5],[0.8],[0.005]])
	#P_right_current = np.array([[0.56],[-0.66],[0.51],[-0.31],[0.83],[-0.29]])
	

# Main portion of code
def main():

	# Initialize node
	rospy.init_node('Home_position')

	# Start Moveit Commander
	InitializeMoveitCommander()
			
	#calibrate both arms home position 
	#indy y abb
	#0degree
	# indyj=[ 0.41858622, -1.26679042, -1.87965003,  0.41952195,  1.57492068,       -1.56783586]
	# abbj=[-7.78692595e-02,  3.23792760e-01,  6.13496452e-01, -1.70074212e-04,        6.34084127e-01, -7.68190648e-02]

	#15 degree
	# indyj=[-0.12341469, -1.26035867, -1.92332673, -0.38586257,  1.61064675,       -1.58584233]
	# abbj=[ 1.58646141e-01,  2.09805506e-01,  7.62483479e-01, -6.99153463e-04,        5.99098122e-01, -1.02028742e-01]

	#30 degree
	# indyj=[ 2.47033419,  1.20919852,  1.16771554, -1.29225056, -1.31398898,        2.29881508]
	# abbj=[-0.09084018,  0.41080732,  0.49139491,  0.00196822,  0.66833665,       -0.61658216]

	#45 degree
	# indyj=[-0.47250318, -1.03370062, -1.13052939,  1.74924782, -1.31226467,        2.52448444]
	# abbj=[-4.97837567e-01,  4.85376254e-01,  1.95404891e-02,  7.31689816e-04,        1.06509409e+00, -1.28401489e+00]

	#60 degree
	indyj=[-0.47797381, -1.2127865 , -1.79595662, -1.52485076,  1.56445979,       -1.43894336]
	abbj=[-4.46826491e-01,  8.36775282e-01, -3.58490688e-01, -4.29344900e-04,        1.09255857e+00, -1.49295508e+00]

	#75 degree
	indyj=[ 2.72480954,  1.47395626,  0.32942789, -1.60618728, -1.7207225 ,        2.90596006]
	abbj=[-5.13370941e-01,  1.17268930e+00, -7.80538866e-01,  3.11292102e-04,        1.17951969e+00, -1.82192567e+00]

	# #-45 degree
	# indyj=[-2.57966512,  1.07278605,  1.01015574, -1.6819083 ,  1.37697027,        3.66511447]
	# abbj=[ 0.19670225,  0.20286928,  0.38896555, -0.00115096,  0.97944137,        0.98185396]

	#maximun
	indyj=[-0.09267645, -1.14888819, -1.98294319, -2.70997071, -1.56212006,        1.56572452]
	abbj=[4.67854646e-01, 1.58698817e-01, 6.21157867e-01, 8.77708766e-04,       7.90790089e-01, 9.91396194e-01]

	home_joints_position = {'joint0': indyj[0], 'joint1': indyj[1], 'joint2': indyj[2], 'joint3': indyj[3], 'joint4': indyj[4], 'joint5': indyj[5], 'joint_1': abbj[0], 'joint_2': abbj[1], 'joint_3': abbj[2], 'joint_4': abbj[3], 'joint_5': abbj[4], 'joint_6': abbj[5]}
	group_both_arms.set_joint_value_target(home_joints_position)
	plan_both = group_both_arms.plan()
	group_both_arms.execute(plan_both)
	rospy.sleep(5)


	# #abb
	# home_joints_position = {'joint_1': abbj[0], 'joint_2': abbj[1], 'joint_3': abbj[2], 'joint_4': abbj[3], 'joint_5': abbj[4], 'joint_6': abbj[5]}
	# group_right_arm.set_joint_value_target(home_joints_position)
	# plan_both = group_right_arm.plan()
	# group_right_arm.execute(plan_both)
	# rospy.sleep(5)

	# #indy7
	# home_joints_position = {'joint0': indyj[0], 'joint1': indyj[1], 'joint2': indyj[2], 'joint3': indyj[3], 'joint4': indyj[4], 'joint5': indyj[5]}
	# group_left_arm.set_joint_value_target(home_joints_position)
	# plan_both = group_left_arm.plan()
	# group_left_arm																						.execute(plan_both)
	# rospy.sleep(5)	

	# dual grasping 0 0 degree
	
	# group_right_arm.set_pose_target([0.35,0.0,0.2,math.pi,0,0],end_effector_link="end_eff_point_vibrationgripper")
	# # group_left_arm.set_pose_target([0.16,-0.11,0.19,3.09387838,0.04260803,0.02662061],end_effector_link="end_eff_point_vibrationgripper")
	# plan_right = group_right_arm.plan()
	# # plan_left = group_left_arm.plan()
	# group_right_arm.execute(plan_right)
	# # group_left_arm.execute(plan_left)
	# rospy.sleep(5)

	# #calibrate both arms home position
	# home_joints_position = {'joint0': 0, 'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0, 'joint5': 0, 'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}
	# group_both_arms.set_joint_value_target(home_joints_position)
	# plan_both = group_both_arms.plan()
	# group_both_arms.execute(plan_both)
	# rospy.sleep(5)

	# group_right_arm.set_pose_target([0,0,0.3,math.pi/2,0,math.pi/2],end_effector_link="end_eff_point_2f")
	# group_left_arm.set_pose_target([0.15,-0.02,0.2,-math.pi,0,0],end_effector_link="end_eff_point_vibrationgripper")
	# plan_right = group_right_arm.plan()
	# plan_left = group_left_arm.plan()
	# group_right_arm.execute(plan_right)
	# group_left_arm.execute(plan_left)
	# rospy.sleep(5)

	# # dual grasping 30 30 degree
	# group_right_arm.set_pose_target([0.4,0,1.0,0,-math.pi/2,2*math.pi/3],end_effector_link="end_eff_point_2f")
	# plan_right = group_right_arm.plan()
	# group_left_arm.set_pose_target([0.4,0,0.9,0,-math.pi/2,-2*math.pi/3],end_effector_link="end_eff_point_vibrationgripper")
	# plan_left = group_left_arm.plan()
	# group_right_arm.execute(plan_right)
	# group_left_arm.execute(plan_left)
	# rospy.sleep(5)

	# #calibrate both arms home position
	# home_joints_position = {'joint0': 0, 'joint1': 0, 'joint2': 0, 'joint3': 0, 'joint4': 0, 'joint5': 0, 'joint_1': 0, 'joint_2': 0, 'joint_3': 0, 'joint_4': 0, 'joint_5': 0, 'joint_6': 0}
	# group_both_arms.set_joint_value_target(home_joints_position)
	# plan_both = group_both_arms.plan()
	# group_both_arms.execute(plan_both)
	# rospy.sleep(5)

	# # dual pusshing
	# # group_right_arm.set_pose_target([0.25,0.2,0.9,0,-math.pi/2,3*math.pi/4],end_effector_link="right_end_effect_point")
	# # plan_right = group_right_arm.plan()
	# # group_left_arm.set_pose_target([0.25,-0.2,0.9,0,-math.pi/2,math.pi],end_effector_link="left_end_effect_point")
	# # plan_left = group_left_arm.plan()
	# # group_right_arm.execute(plan_right)
	# # group_left_arm.execute(plan_left)
	# # rospy.sleep(5)


    




if __name__ == '__main__':
	main()
