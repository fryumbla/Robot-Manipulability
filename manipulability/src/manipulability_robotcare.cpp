#include "ros/ros.h"
#include <iostream> 
#include <fstream>

// estas librerias es para los objetos moveit_msgs::CollisionObject t de moveit
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometric_shapes/shape_operations.h"


#include <math.h>

// esta es para vizualizar en rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/tf.h>


#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include "manipulability_msgs/manipulability.h"

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

using namespace std;

ros::NodeHandle *node_handle = NULL;

std::vector<geometry_msgs::Pose> list;
// For visualizing things in rviz

rviz_visual_tools::RvizVisualToolsPtr visual_tools;

robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* group_right;
robot_state::JointModelGroup* group_left;

double manipulability_right()
{
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(group_right,
                              kinematic_state->getLinkModel(group_right->getLinkModelNames().back()),
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
  kinematics_metrics_->getManipulability(*state, group_right, manipulability);
  kinematics_metrics_->getManipulabilityIndex(*state, group_right, manipulability_index);  
  // kinematics_metrics_->getManipulabilityEllipsoid(*state, group_right, eigen_values,eigen_vectors);

  return manipulability;
}

double manipulability_left()
{
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(group_left,
                              kinematic_state->getLinkModel(group_left->getLinkModelNames().back()),
                              reference_point_position, jacobian);

  // cout << "Jacobian: \n" << jacobian << "\n\n";

  Eigen::MatrixXd jjt;
  jjt= (jacobian*jacobian.transpose());
  double w;
  w= sqrt(jjt.determinant());
  cout << "w: " << w << "\n\n";

  double manipulability_index, manipulability;

  // kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  // kinematics_metrics_.reset(new kinematics_metrics::KinematicsMetrics(kinematic_model));

  // // Eigen::MatrixXcd eigen_values,eigen_vectors;
  
  // robot_state::RobotState *state = kinematic_state.get();
  // kinematics_metrics_->getManipulability(*state, group_left, manipulability);
  // kinematics_metrics_->getManipulabilityIndex(*state, group_left, manipulability_index);  
  // kinematics_metrics_->getManipulabilityEllipsoid(*state, group_right, eigen_values,eigen_vectors);

  return w;
}


//get the inverse kinematics of the arm for the
//input eef pose
bool getIK_right(geometry_msgs::Pose eef_pose_right)
{
  kinematic_state->enforceBounds();

  Eigen::Isometry3d pose_right;
  tf::poseMsgToEigen(eef_pose_right, pose_right);
  // cout << "Pose: " << eef_pose << "\n";
  
  bool found_ik =kinematic_state->setFromIK(group_right, pose_right, 10, 0.5);

  return found_ik;

}
bool getIK_left(geometry_msgs::Pose eef_pose_left)
{
  kinematic_state->enforceBounds();

  Eigen::Isometry3d pose_left;
  tf::poseMsgToEigen(eef_pose_left, pose_left);
  // cout << "Pose: " << eef_pose_left << "\n";

  bool found_ik =kinematic_state->setFromIK(group_left, pose_left, 10, 0.5);

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
    kinematic_state->copyJointGroupPositions(group_right, joint_values);
    kinematic_state->copyJointGroupPositions(group_left, joint_values);

    for(std::size_t i = 0; i < joint_values.size(); i++){
        joint_values[i] = 0;
    }
    kinematic_state->setJointGroupPositions(group_right, joint_values);
    kinematic_state->setJointGroupPositions(group_left, joint_values);

}

//initializations might be needed though
void initialize()
{

  ROS_INFO("RobotCare Robot Manipulability");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();

  kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
  kinematic_state->enforceBounds();

  group_right = kinematic_model->getJointModelGroup("right_arm");
  group_right->getJointModelNames();
  
  group_left = kinematic_model->getJointModelGroup("left_arm");
  group_left->getJointModelNames();

  reset_joint_values();

}

struct Point {
     Point( double X, double Y ): x(X), y(Y) {};
     double x;
     double y;
};//Simple Point structure

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manipulability_skku_robot");
	ros::NodeHandle n;
	node_handle = &n;
  ros::Publisher robot_state_pub;
  robot_state_pub   = n.advertise<manipulability_msgs::manipulability>("/manipulability_data",1);

  manipulability_msgs::manipulability manipulability;
  
  manipulability.header.frame_id="Manipulability_data";

  sensor_msgs::JointState joint_values_right;
  sensor_msgs::JointState joint_values_left;
  
    // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_link","/visualization_marker_array"));
  visual_tools->loadMarkerPub();


  // Clear messages
  visual_tools->deleteAllMarkers();
  visual_tools->enableBatchPublishing();

  initialize();

  std::ofstream myFile("manipulability.csv");

    geometry_msgs::Pose pose_right, pose_left;  

    double step=0.03;
    // double sizex= 0.5, sizey=0.55;
    double sizex= 0.4, sizey=0.30;
    
    for (double i = 0.1; i <= sizex; i=i+step)
    {
      cout << "x:" << i << "\n";
        for (double j = -0.5; j <= 0.2; j=j+step)
        {
          cout << "y:" << j << "\n";
            // for (double k = 0.7; k <= 0.95; k=k+step)
            // {

              for (double m = 0.0; m <= 1.57; m=m+0.1745329252)
              { 
                manipulability.header.stamp=ros::Time::now();
              
                geometry_msgs::Pose pose;
                pose_right = create_pose(i,j,0.8,-3.1416+m,-1.57,0);
                pose_left = create_pose(i,j,0.8,m,1.57,0);
                                // left  x angel 0 to 1.57


                if (getIK_right(pose_right))
                {
                    manipulability.pose = pose_right;
                  // Retrieve joint values map
                  myFile << i << "," << j << "," << "0.8" << "," << m << "," ;
                  std::map<std::string, double> goal_joint_values;
                  std::vector<double> iiwa_joint_values;
                  kinematic_state->copyJointGroupPositions(group_right, iiwa_joint_values);
                  const std::vector<std::string> &iiwa_joint_names = group_right->getJointModelNames();
                  for(std::size_t i = 0; i < iiwa_joint_names.size(); ++i) 
                  { // TODO cleanly remove the fixed joints (first and last) from the list
                      
                      goal_joint_values[iiwa_joint_names[i]] = iiwa_joint_values[i];
                      myFile << iiwa_joint_values[i] << "," ;
                      // cout << iiwa_joint_names[i] << " = " << iiwa_joint_values[i] << "\n";
                  }
                  
                  joint_values_right.header.stamp=ros::Time::now();
                  joint_values_right.header.frame_id="right_arm";
                  joint_values_right.name = group_right->getJointModelNames();
                  joint_values_right.position=iiwa_joint_values;



                  const Eigen::Affine3d &eef_state = kinematic_state->getGlobalLinkTransform("right_end_effect_point");
                  geometry_msgs::Pose eef_pose, eef_final;
                  tf::poseEigenToMsg(eef_state, eef_pose);
                  tf::poseEigenToMsg(eef_state, eef_final);
                  // cout << eef_final.orientation << "\n";
                  eef_final.orientation.x=-eef_final.orientation.x;
                  eef_final.orientation.y=-eef_final.orientation.y;
                  eef_final.orientation.z=-eef_final.orientation.z;
                  eef_final.orientation.w=-eef_final.orientation.w;

                  // cout << eef_pose << "\n";

                  manipulability.joint_values= joint_values_right;
                  double w= manipulability_right();
                  cout << "right manipulability:" << w << "\n";
                  manipulability.manipulability = w;
                  // drawing points
                  geometry_msgs::Vector3 scale = visual_tools->getScale(rviz_visual_tools::SMALL);
                  std_msgs::ColorRGBA color = visual_tools->getColorScale(w*10);
                  visual_tools->publishSphere(visual_tools->convertPose(pose_right), color, scale, "Sphere");
                  // visual_tools->publishZArrow(visual_tools->convertPose(eef_final), rviz_visual_tools::ORANGE, rviz_visual_tools::XXXXSMALL);
                  // visual_tools->publishLine(visual_tools->convertPose(eef_pose), visual_tools->convertPose(eef_final), rviz_visual_tools::RAND);
                  visual_tools->publishAxis(visual_tools->convertPose(pose_right), rviz_visual_tools::XXXXSMALL);
                  visual_tools->trigger();
                  robot_state_pub.publish(manipulability);


                  // myFile << i << "," << j << "," << "0.8" << "," << m << "," << w << "\n";
                  myFile << w << "\n";






                }else
                {
                cout << "No found right IK\n";
                }

                // if (getIK_left(pose_left))
                // {
                //   manipulability.pose = pose_left;
                // // Retrieve joint values map
                // std::map<std::string, double> goal_joint_values;
                // std::vector<double> iiwa_joint_values;
                // kinematic_state->copyJointGroupPositions(group_left, iiwa_joint_values);
                // const std::vector<std::string> &iiwa_joint_names = group_left->getJointModelNames();
                // for(std::size_t i = 0; i < iiwa_joint_names.size(); ++i) 
                // { // TODO cleanly remove the fixed joints (first and last) from the list
                //     goal_joint_values[iiwa_joint_names[i]] = iiwa_joint_values[i];
                //     // cout << iiwa_joint_names[i] << " = " << iiwa_joint_values[i] << "\n";
                // }

                // joint_values_left.header.stamp=ros::Time::now();
                // joint_values_left.header.frame_id="left_arm";
                // joint_values_left.name = group_left->getJointModelNames();
                // joint_values_left.position=iiwa_joint_values;
                
                // const Eigen::Affine3d &eef_state = kinematic_state->getGlobalLinkTransform("left_end_effect_point");
                // geometry_msgs::Pose eef_pose, eef_final;
                // tf::poseEigenToMsg(eef_state, eef_pose);
                // tf::poseEigenToMsg(eef_state, eef_final);
                // eef_final.position.x=eef_final.position.x+0.05;
                // eef_final.position.y=eef_final.position.y+0.05;
                // eef_final.position.z=eef_final.position.z+0.05;
                // // cout << eef_pose << "\n";
                

                // manipulability.joint_values= joint_values_left;

                // double w= manipulability_left();
                // cout << "left manipulability: " << w << "\n";
                // manipulability.manipulability = w;
                // // drawing points
                // geometry_msgs::Vector3 scale = visual_tools->getScale(rviz_visual_tools::SMALL);
                // std_msgs::ColorRGBA color = visual_tools->getColorScale(w*10);
                // visual_tools->publishSphere(visual_tools->convertPose(eef_pose), color, scale, "Sphere");
                // // visual_tools->publishArrow(visual_tools->convertPose(eef_pose),visual_tools->convertPose(eef_final), color, scale);
                // visual_tools->publishAxis(visual_tools->convertPose(eef_pose), rviz_visual_tools::XXXXSMALL);
                
                // visual_tools->trigger();

                // robot_state_pub.publish(manipulability);
                
                // }else
                // {
                // cout << "No found left IK\n";
                // }

              }

            // }
        }
    
   
    }
  
  // Close the file
    myFile.close();

  // ros::spin();
  ros::shutdown();  

 return 0;
}
