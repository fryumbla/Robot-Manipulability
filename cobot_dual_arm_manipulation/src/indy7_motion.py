#!/usr/bin/env python


import numpy as np
import rospy
import tf
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import rospkg
import os


from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from moveit_commander import MoveGroupCommander


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

	group_left_arm = MoveGroupCommander("indy7")
	group_left_arm.set_goal_position_tolerance(0.001)
	group_left_arm.set_goal_orientation_tolerance(0.001)
	group_left_arm.set_planning_time(5.0)

	#We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
	# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	# Obtain current poses of left and right end-effectors: [x,y,z,roll,pitch,yaw].T
	P_left_pose = group_left_arm.get_current_pose()
	# P_right_pose = group_right_arm.get_current_pose()
	P_left_euler = group_left_arm.get_current_rpy()
	# P_right_euler = group_right_arm.get_current_rpy()

	global P_left_current, P_right_current
	P_left_current = np.array([[P_left_pose.pose.position.x],[P_left_pose.pose.position.y],[P_left_pose.pose.position.z],[P_left_euler[0]],[P_left_euler[1]],[P_left_euler[2]]])
	
	print(P_left_current)
	
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
	jointindy.append([ 0.41858622, -1.26679042, -1.87965003,  0.41952195,  1.57492068,       -1.56783586])
	jointabb.append([-7.78692595e-02,  3.23792760e-01,  6.13496452e-01, -1.70074212e-04,        6.34084127e-01, -7.68190648e-02])

	#15 degree
	jointindy.append([-0.12341469, -1.26035867, -1.92332673, -0.38586257,  1.61064675,       -1.58584233])
	jointabb.append([ 1.58646141e-01,  2.09805506e-01,  7.62483479e-01, -6.99153463e-04,        5.99098122e-01, -1.02028742e-01])

	#30 degree
	jointindy.append([ 2.47033419,  1.20919852,  1.16771554, -1.29225056, -1.31398898,        2.29881508])
	jointabb.append([-0.09084018,  0.41080732,  0.49139491,  0.00196822,  0.66833665,       -0.61658216])

	#45 degree
	jointindy.append([-0.47250318, -1.03370062, -1.13052939,  1.74924782, -1.31226467,        2.52448444])
	jointabb.append([-4.97837567e-01,  4.85376254e-01,  1.95404891e-02,  7.31689816e-04,        1.06509409e+00, -1.28401489e+00])

	#60 degree
	jointindy.append([-0.47797381, -1.2127865 , -1.79595662, -1.52485076,  1.56445979,       -1.43894336])
	jointabb.append([-4.46826491e-01,  8.36775282e-01, -3.58490688e-01, -4.29344900e-04,        1.09255857e+00, -1.49295508e+00])

	#75 degree
	jointindy.append([ 2.72480954,  1.47395626,  0.32942789, -1.60618728, -1.7207225 ,        2.90596006])
	jointabb.append([-5.13370941e-01,  1.17268930e+00, -7.80538866e-01,  3.11292102e-04,        1.17951969e+00, -1.82192567e+00])

	# #-45 degree
	# jointindy[p]=[-2.57966512,  1.07278605,  1.01015574, -1.6819083 ,  1.37697027,        3.66511447]
	# jointabb[p]=[ 0.19670225,  0.20286928,  0.38896555, -0.00115096,  0.97944137,        0.98185396]

	#maximun
	jointindy.append([-0.09267645, -1.14888819, -1.98294319, -2.70997071, -1.56212006,        1.56572452])
	jointabb.append([4.67854646e-01, 1.58698817e-01, 6.21157867e-01, 8.77708766e-04,       7.90790089e-01, 9.91396194e-01])
	while not rospy.is_shutdown():

		number = input ("Enter number: ")

		if (number==0):
			# group_left_arm.set_pose_target([-0.299935650695,-0.174063883192,0.30705390301,0,math.pi,math.pi/2],end_effector_link="end_eff_point_2f")
			# plan_left = group_left_arm.plan()
			# group_left_arm.execute(plan_left)
			home_joints_position = {'joint0': 0, 'joint1': 0, 'joint2': -3.1416/2, 'joint3': 0, 'joint4': -3.1416/2, 'joint5': -3.1416/2}
			group_left_arm.set_joint_value_target(home_joints_position)
			plan_left = group_left_arm.plan()
			group_left_arm.execute(plan_left)

		if (number>=1):		
			home_joints_position = {'joint0': jointindy[number-1][0], 'joint1': jointindy[number-1][1], 'joint2': jointindy[number-1][2], 'joint3': jointindy[number-1][3], 'joint4': jointindy[number-1][4], 'joint5': jointindy[number-1][5]}
			group_left_arm.set_joint_value_target(home_joints_position)
			plan_left = group_left_arm.plan()
			group_left_arm.execute(plan_left)



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
