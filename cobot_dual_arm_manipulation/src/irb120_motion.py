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
	robot = moveit_commander.RobotCommander()

	# We can get a list of all the groups in the robot:
	print("============ Available Planning Groups:", robot.get_group_names())

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

	group_right_arm = MoveGroupCommander("irb120")
	group_right_arm.set_goal_position_tolerance(0.001)
	group_right_arm.set_goal_orientation_tolerance(0.001)
	group_right_arm.set_planning_time(5.0)

	#We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
	# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	# Obtain current poses of left and right end-effectors: [x,y,z,roll,pitch,yaw].T
	# P_left_pose = group_left_arm.get_current_pose()
	P_right_pose = group_right_arm.get_current_pose()
	# P_left_euler = group_left_arm.get_current_rpy()
	P_right_euler = group_right_arm.get_current_rpy()

	# global P_left_current, P_right_current
	# P_left_current = np.array([[P_left_pose.pose.position.x],[P_left_pose.pose.position.y],[P_left_pose.pose.position.z],[P_left_euler[0]],[P_left_euler[1]],[P_left_euler[2]]])
	P_right_current = np.array([[P_right_pose.pose.position.x],[P_right_pose.pose.position.y],[P_right_pose.pose.position.z],[P_right_euler[0]],[P_right_euler[1]],[P_right_euler[2]]])
	print P_right_current
	print group_right_arm.get_current_joint_values()
	#P_left_current = np.array([[0.18],[0.64],[-0.16],[-0.5],[0.8],[0.005]])
	#P_right_current = np.array([[0.56],[-0.66],[0.51],[-0.31],[0.83],[-0.29]])
	

# Main portion of code
def main():

	# Initialize node
	rospy.init_node('Home_position')

	# Start Moveit Commander
	InitializeMoveitCommander()

	jointindy=[]
	jointabb=[]
			
	#calibrate both arms home position 
	#indy y abb
	#0degree
	jointindy.append([ 0.55282767, -1.23373548, -1.64071599,  0.56851026,  1.34317107,       -1.71251893])
	jointabb.append([-0.43376803,  0.21452732,  0.6986698 ,  0.00135015,  0.65691976,       -0.43453093])

	#15 degree
	jointindy.append([-0.12341469, -1.26035867, -1.92332673, -0.38586257,  1.61064675,       -1.58584233])
	jointabb.append([ 0.15828575,  0.17123591,  0.75151993,  0.00140375,  0.64810221,       -0.10487191])

	#30 degree
	jointindy.append([ 2.52597801,  1.1964157 ,  1.25312434, -1.22923342, -1.29999417,        2.21556189])
	jointabb.append([-2.30210457e-01,  4.04312102e-01,  4.45723573e-01, -5.76174265e-04,        7.21467864e-01, -7.52651898e-01])

	#45 degree
	jointindy.append([-0.33345942, -1.12338211, -1.81526974, -1.12700921,  1.48183835,       -1.38695455])
	jointabb.append([-4.41114165e-01,  2.32003334e-01,  3.08185713e-01, -3.14825886e-04,        1.03086119e+00, -1.22547914e+00])

	#60 degree
	jointindy.append([-0.47797381, -1.2127865 , -1.79595662, -1.52485076,  1.56445979,       -1.43894336])
	jointabb.append([-4.46732325e-01,  8.23023075e-01, -3.80063987e-01, -9.85627583e-04,        1.12783578e+00, -1.49312957e+00])

	#75 degree
	jointindy.append([ 2.66203795,  1.34301007,  0.3269632 , -1.59217244, -1.78843362,        3.04053427])
	jointabb.append([-4.31418180e-01,  9.65939036e-01, -7.83209640e-01, -5.02259610e-04,        1.38725303e+00, -1.73978354e+00])

	# #-45 degree
	# jointindy[p]=[-2.57966512,  1.07278605,  1.01015574, -1.6819083 ,  1.37697027,        3.66511447]
	# jointabb[p]=[ 0.19670225,  0.20286928,  0.38896555, -0.00115096,  0.97944137,        0.98185396]

	#maximun
	jointindy.append([-0.09267645, -1.14888819, -1.98294319, -2.70997071, -1.56212006,        1.56572452])
	jointabb.append([4.67957847e-01, 1.31391000e-01, 6.02237645e-01, 6.29703752e-04,       8.36352136e-01, 9.91505765e-01])

	
	while not rospy.is_shutdown():

		number = input ("Enter number: ")

		if (number==0):
			group_right_arm.set_pose_target([0.34,-2.49813353e-02,5.33612867e-02,math.pi,0,0],end_effector_link="end_eff_point_vibrationgripper")
			plan_right = group_right_arm.plan()
			group_right_arm.execute(plan_right)
			# home_joints_position = {'joint0': 0, 'joint1': 0, 'joint2': -3.1416/2, 'joint3': 0, 'joint4': -3.1416/2, 'joint5': -3.1416/2}
			# group_right_arm.set_joint_value_target(home_joints_position)
			# plan_right = group_right_arm.plan()
			group_right_arm.execute(plan_right)

		if (number>=1 and number<=9):		
			home_joints_position = {'joint_1': jointabb[number-1][0], 'joint_2': jointabb[number-1][1], 'joint_3': jointabb[number-1][2], 'joint_4': jointabb[number-1][3], 'joint_5': jointabb[number-1][4], 'joint_6': jointabb[number-1][5]}
			group_right_arm.set_joint_value_target(home_joints_position)
			plan_right = group_right_arm.plan()
			group_right_arm.execute(plan_right)

		#aproximamos al connector 
		if (number==10):
			mating_pose = group_right_arm.get_current_pose().pose.position
			mating_euler = group_right_arm.get_current_rpy()
			group_right_arm.set_pose_target([mating_pose.x-0.042,mating_pose.y,mating_pose.z,mating_euler[0],mating_euler[1],mating_euler[2]],end_effector_link="end_eff_point_vibrationgripper")
			plan_right = group_right_arm.plan()
			group_right_arm.execute(plan_right)

		#mating y despues debemos regresar al punto inicial
		if (number==11):
			mating_pose = group_right_arm.get_current_pose().pose.position
			mating_euler = group_right_arm.get_current_rpy()
			group_right_arm.set_pose_target([mating_pose.x-0.012,mating_pose.y,mating_pose.z,mating_euler[0],mating_euler[1],mating_euler[2]],end_effector_link="end_eff_point_vibrationgripper")
			plan_right = group_right_arm.plan()
			group_right_arm.execute(plan_right)

		if (number==12):
			group_right_arm.set_pose_target([0.25,-0.1,0.15,math.pi,0,-5.24079111e-01],end_effector_link="end_eff_point_vibrationgripper")
			plan_right = group_right_arm.plan()
			group_right_arm.execute(plan_right)

	# #abb
	# home_joints_position = {'joint_1': jointabb[p][0], 'joint_2': jointabb[p][1], 'joint_3': jointabb[p][2], 'joint_4': jointabb[p][3], 'joint_5': jointabb[p][4], 'joint_6': jointabb[p][5]}
	# group_right_arm.set_joint_value_target(home_joints_position)
	# plan_both = group_right_arm.plan()
	# group_right_arm.execute(plan_both)
	# rospy.sleep(5)

	# #indy7
	# home_joints_position = {'joint0': jointindy[p][0], 'joint1': jointindy[p][1], 'joint2': jointindy[p][2], 'joint3': jointindy[p][3], 'joint4': jointindy[p][4], 'joint5': jointindy[p][5]}
	# group_left_arm.set_joint_value_target(home_joints_position)
	# plan_both = group_left_arm.plan()
	# group_left_arm																						.execute(plan_both)
	# rospy.sleep(5)	



if __name__ == '__main__':
	main()
