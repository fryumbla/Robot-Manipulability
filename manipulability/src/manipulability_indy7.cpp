#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <rviz_visual_tools/rviz_visual_tools.h>


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

ros::NodeHandle *node_handle = NULL;
tf::TransformListener *tf_listener = NULL;

robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_model_group;

double manipulability()
{
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                              kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                              reference_point_position, jacobian);

  // cout << "Jacobian: \n" << jacobian << "\n\n";
  // Eigen::MatrixXd jjt;
  // jjt= (jacobian*jacobian.transpose());
  // double w;
  // w= sqrt(jjt.determinant());
  // cout << "w: " << w << "\n\n";

  double manipulability_index, manipulability;

  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(kinematic_model));

  // Eigen::MatrixXcd eigen_values,eigen_vectors;
  
  robot_state::RobotState *state = kinematic_state.get();
  kinematics_metrics_->getManipulability(*state, joint_model_group, manipulability);
  kinematics_metrics_->getManipulabilityIndex(*state, joint_model_group, manipulability_index);  
  // kinematics_metrics_->getManipulabilityEllipsoid(*state, joint_model_group, eigen_values,eigen_vectors);

  return manipulability;

}

//get the inverse kinematics of the arm for the
//input eef pose
bool getIK(geometry_msgs::Pose eef_pose)
{
  kinematic_state->enforceBounds();
  Eigen::Isometry3d pose_in;
  tf::poseMsgToEigen(eef_pose, pose_in);
  // cout << "Pose: " << eef_pose << "\n";
  bool found_ik =kinematic_state->setFromIK(joint_model_group, pose_in, 10, 0.5);

  return found_ik;
}

geometry_msgs::Pose create_pose(double pos_x, double pos_y, double pos_z, double orient_x, double orient_y, double orient_z)
{
  geometry_msgs::Pose pose;
  pose.position.x = pos_x;
  pose.position.y = pos_y;
  pose.position.z = pos_z;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(orient_x,orient_y,orient_z);
  return pose;
}

//reset the values that the arm is currently at - set to home
void reset_joint_values(){

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_values.size(); i++){
        joint_values[i] = 0;
    }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
}

//init function, shouldn't need to be modified, additional
//initializations might be needed though
void initialize()
{
  ROS_INFO("instantiating Schunk arm");
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();

  kinematic_state.reset(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
  kinematic_state->enforceBounds();

  joint_model_group = kinematic_model->getJointModelGroup("indy7");
  joint_model_group->getJointModelNames();

  reset_joint_values();

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fran");
	ros::NodeHandle n;
	node_handle = &n;
  
    // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("ground","/visualization_marker_array"));
  visual_tools->loadMarkerPub();

  // Clear messages
  visual_tools->deleteAllMarkers();
  visual_tools->enableBatchPublishing();

  initialize();
  
  double step=0.05;
  double sizex= 0.5, sizey=0.5;
  
  for (double i = -sizex; i < sizex; i=i+step)
  {
    for (double j = -sizey; j < sizey; j=j+step)
    {
      for (double k = 0.3; k < 1.4; k=k+step)
      {
        geometry_msgs::Pose pose;
        pose = create_pose(i,j,k,0,0,0);


        if (getIK(pose))
        {
          // Retrieve joint values map
          std::map<std::string, double> goal_joint_values;
          std::vector<double> iiwa_joint_values;
          kinematic_state->copyJointGroupPositions(joint_model_group, iiwa_joint_values);
          const std::vector<std::string> &iiwa_joint_names = joint_model_group->getJointModelNames();
          for(std::size_t i = 0; i < iiwa_joint_names.size(); ++i) 
          { // TODO cleanly remove the fixed joints (first and last) from the list
            goal_joint_values[iiwa_joint_names[i]] = iiwa_joint_values[i];
            // cout << iiwa_joint_names[i] << " = " << iiwa_joint_values[i] << "\n";
          }

          const Eigen::Affine3d &eef_state = kinematic_state->getGlobalLinkTransform("link6");
          geometry_msgs::Pose eef_pose;
          tf::poseEigenToMsg(eef_state, eef_pose);
          // cout << eef_pose << "\n";

          double w= manipulability();
          cout << w << "\n";

          // drawing points
          geometry_msgs::Vector3 scale = visual_tools->getScale(rviz_visual_tools::SMALL);
          std_msgs::ColorRGBA color = visual_tools->getColorScale(w*10);
          visual_tools->publishSphere(visual_tools->convertPose(eef_pose), color, scale, "Sphere");
        
          visual_tools->trigger();
          
        }else
        {
          cout << "No found IK\n";
        }

      }
      
    }
     


  }
  
  // ros::spin();
  ros::shutdown();  
 return 0;
}
