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
	jointindy.append([ 0.0537188 , -1.26572724, -1.83837644,  0.05281199,  1.53318763,       -1.57330448])
	jointabb.append([0.43317108, 0.21475959, 0.69859389, 0.00180723, 0.65746665,       0.43157438])

	#15 degree
	jointindy.append([ 0.12001539, -1.14730998, -1.9870047 , -0.14101459,  1.56308757,       -1.5696963 ])
	jointabb.append([-1.94439444e-01,  5.84397388e-02,  6.81637565e-01, -3.43987090e-05,        8.31548822e-01, -4.56587065e-01])

	#30 degree
	jointindy.append([-0.06613258, -1.08680787, -1.86740436, -0.5986809 ,  1.41634728,       -1.46462332])
	jointabb.append([-4.87358407e-01, -4.22201894e-02,  6.06312848e-01,  5.47445273e-04,        1.00577221e+00, -1.01052963e+00])

	#45 degree
	jointindy.append([-0.5358581 , -1.15466513, -1.01414956,  1.71242838, -1.36575865,        2.52852364])
	jointabb.append([-3.88253294e-01,  4.56915362e-01,  1.85277813e-01,  8.19518533e-04,        9.29294445e-01, -1.17451908e+00])

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
