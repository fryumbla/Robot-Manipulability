#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// #include <rviz_visual_tools/rviz_visual_tools.h>

#include <iostream> 
#include <vector> 

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/JointState.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>


#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::Vector3d;

using namespace std;

std::string iiwa_ee_link = "6_link"; // TODO should be iiwa_link_ee, but then results are even worse.
std::string linkgroup = "arm";

ros::NodeHandle *node_handle = NULL;
moveit::planning_interface::MoveGroupInterface *group = NULL;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface = NULL;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* iiwa_modelgroup;
robot_model::RobotModelPtr kinematic_model;
tf::TransformListener *tf_listener = NULL;

void reply(const sensor_msgs::JointState& data){

		
	// Retrieve position from message
	geometry_msgs::Pose target_position1; // Expressed in group->getPlanningFrame()
	target_position1.position.x = 0.05;
	target_position1.position.y = 0.05;
	target_position1.position.z = 0.7;
	target_position1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
	
    cout << target_position1 << "\n";
    
    const Eigen::Affine3d &eef_state = kinematic_state->getGlobalLinkTransform("6_link");
    geometry_msgs::Pose eef_pose;
    tf::poseEigenToMsg(eef_state, eef_pose);

    cout << eef_pose << "\n";

	// Compute IK
	Eigen::Isometry3d target_eig_pose1;
	tf::poseMsgToEigen(target_position1, target_eig_pose1);	
	kinematic_state->setToIKSolverFrame(target_eig_pose1, group->getPlanningFrame()); // Convert to robotmodel frame if needed.
	bool found_ik = kinematic_state->setFromIK(iiwa_modelgroup, target_eig_pose1, iiwa_ee_link, 100, 0.1);
    cout << found_ik << "\n";
	if(!found_ik)
		return;
    
    cout << "hola" << "\n\n";
	
	// Retrieve joint values map
	std::map<std::string, double> goal_joint_values;
	std::vector<double> iiwa_joint_values;
	kinematic_state->copyJointGroupPositions(iiwa_modelgroup, iiwa_joint_values);
	const std::vector<std::string> &iiwa_joint_names = iiwa_modelgroup->getJointModelNames();
	for(std::size_t i = 0; i < iiwa_joint_names.size(); ++i) { // TODO cleanly remove the fixed joints (first and last) from the list
		goal_joint_values[iiwa_joint_names[i]] = iiwa_joint_values[i];
        cout << iiwa_joint_names[i] << iiwa_joint_values[i] << "\n\n";
	}
	
	// Set plan request data
	// group->setJointValueTarget(goal_joint_values);
    //	group->setPoseTarget(target_position1, iiwa_ee_link); // This works perfectly, but not supported by STOMP...
	
	robot_state::RobotState start_state(*group->getCurrentState());
	group->setStartState(start_state);	
	group->setGoalOrientationTolerance(0.01);
	group->setGoalPositionTolerance(0.01);

	// Plan and execute
	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// bool success = group->plan(my_plan);
    
	// group->execute(my_plan);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "moveit_iiwa_listening");
	ros::NodeHandle n;
	node_handle = &n;
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	kinematic_state.reset(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	iiwa_modelgroup = kinematic_model->getJointModelGroup(linkgroup);


	
	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	
	moveit::planning_interface::PlanningSceneInterface psi;
	planning_scene_interface = &psi;
	moveit::planning_interface::MoveGroupInterface g(linkgroup);
	group = &g;
	
	ros::Subscriber sub = node_handle->subscribe("/joint_states", 000, reply);

	ros::spin();
	
	ros::shutdown();

	return 0;
}