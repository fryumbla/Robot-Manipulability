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
	group_both_arms.set_goal_position_tolerance(0.001)
	group_both_arms.set_goal_orientation_tolerance(0.001)
	group_both_arms.set_planning_time(5.0)

	group_left_arm = MoveGroupCommander("indy7")
	group_left_arm.set_goal_position_tolerance(0.001)
	group_left_arm.set_goal_orientation_tolerance(0.001)
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
	rospy.init_node('Dual_arm_motion')

	# Start Moveit Commander
	InitializeMoveitCommander()

	jointindy=[]
	jointabb=[]
			
	#calibrate both arms home position 
	#indy y abb
	#0degree
	jointindy.append([ 0.0537188 , -1.26572724, -1.83837644,  0.05281199,  1.53318763,       -1.57330448])
	jointabb.append([0.43317108, 0.21475959, 0.69859389, 0.00180723, 0.65746665,       0.43157438])

	#15 degree
	jointindy.append([ 0.12001539, -1.14730998, -1.9870047 , -0.14101459,  1.56308757,       -1.5696963 ])
	jointabb.append([-1.94439444e-01,  5.84397388e-02,  6.81637565e-01, -3.43987090e-05,        8.31548822e-01, -4.56587065e-01])

	#30 degree
	jointindy.append([-0.06613258, -1.08680787, -1.86740436, -0.5986809 ,  1.41634728,       -1.46462332])
	jointabb.append([-4.87358407e-01, -4.22201894e-02,  6.06312848e-01,  5.47445273e-04,        1.00577221e+00, -1.01052963e+00])

	#45 degree
	jointindy.append([-0.53253617, -1.05781852, -1.0534374 ,  1.70173515, -1.35475633,        2.58659102])
	jointabb.append([-3.88261497e-01,  3.94324265e-01,  1.00460579e-01,  5.87325229e-04,        1.07472830e+00, -1.17407567e+00])

	#60 degree
	jointindy.append([-0.46207666, -1.18946499, -0.69725908,  1.59105471, -1.51190398,        2.82531346])
	jointabb.append([-4.94380741e-01,  5.95052149e-01, -1.85281435e-01, -8.44017968e-05,        1.16007364e+00, -1.54235530e+00])

	# #75 degree
	# jointindy.append([ 2.62394199,  2.1569631 , -1.15040169, -1.43246986, -1.78557209,       -2.56215665])
	# jointabb.append([-4.31454170e-01,  1.02330574e+00, -5.57752243e-01,  1.21944577e-04,        1.10624058e+00, -1.74020412e+00])

	# #-45 degree
	# jointindy[p]=[-2.57966512,  1.07278605,  1.01015574, -1.6819083 ,  1.37697027,        3.66511447]
	# jointabb[p]=[ 0.19670225,  0.20286928,  0.38896555, -0.00115096,  0.97944137,        0.98185396]

	#maximun
	jointindy.append([-0.09267645, -1.14888819, -1.98294319, -2.70997071, -1.56212006,        1.56572452])
	jointabb.append([4.67957847e-01, 1.31391000e-01, 6.02237645e-01, 6.29703752e-04,       8.36352136e-01, 9.91505765e-01])

	while not rospy.is_shutdown():

		number = input ("Enter number: ")

		if (number==0):
			group_right_arm.set_pose_target([0.349713119182,-0.0249854954002,0.0533491845196,math.pi,0,0],end_effector_link="end_eff_point_vibrationgripper")
			group_left_arm.set_pose_target([-0.299935650695,-0.174063883192,0.30705390301,0,math.pi,math.pi/2],end_effector_link="end_eff_point_2f")
			plan_right = group_right_arm.plan()
			plan_left = group_left_arm.plan()
			group_right_arm.execute(plan_right)
			group_left_arm.execute(plan_left)


		if (number>=1):	
			home_joints_position = {'joint0': jointindy[number-1][0], 'joint1': jointindy[number-1][1], 'joint2': jointindy[number-1][2], 'joint3': jointindy[number-1][3], 'joint4': jointindy[number-1][4], 'joint5': jointindy[number-1][5], 
									'joint_1': jointabb[number-1][0], 'joint_2': jointabb[number-1][1], 'joint_3': jointabb[number-1][2], 'joint_4': jointabb[number-1][3], 'joint_5': jointabb[number-1][4], 'joint_6': jointabb[number-1][5]}
			group_both_arms.set_joint_value_target(home_joints_position)
			plan_both = group_both_arms.plan()
			group_both_arms.execute(plan_both)


	# for p in xrange(0,len(jointindy)):
	# 	print("Position %d"%p)




	# rospy.sleep(5)
	# group_right_arm.set_pose_target([0.349713119182,-0.0249854954002,0.0533491845196,math.pi,0,0],end_effector_link="end_eff_point_vibrationgripper")
	# group_left_arm.set_pose_target([-0.299935650695,-0.174063883192,0.30705390301,0,math.pi,math.pi/2],end_effector_link="end_eff_point_2f")
	# plan_right = group_right_arm.plan()
	# plan_left = group_left_arm.plan()
	# group_right_arm.execute(plan_right)
	# group_left_arm.execute(plan_left)
	# rospy.sleep(5)


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
