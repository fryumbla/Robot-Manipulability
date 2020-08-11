#include "ros/ros.h"
#include <iostream> 

// estas librerias es para los objetos moveit_msgs::CollisionObject t de moveit
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometric_shapes/shape_operations.h"


#include <math.h>

// esta es para vizualizar en rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/tf.h>


#include <geometry_msgs/Point.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>




ros::NodeHandle *node_handle = NULL;

std::vector<geometry_msgs::Pose> list;
// For visualizing things in rviz

rviz_visual_tools::RvizVisualToolsPtr visual_tools;

robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* group_right;
robot_state::JointModelGroup* group_left;




double range = 0.1;

void box(moveit_msgs::CollisionObject box)
{
    std::cout << "Recognition box" << std::endl;
    
    double dx = box.primitives[0].dimensions[0];
    double dy = box.primitives[0].dimensions[1];
    double dz = box.primitives[0].dimensions[2];

    geometry_msgs::Point p;
    p.x = box.primitive_poses[0].position.x;
    p.y = box.primitive_poses[0].position.y;
    p.z = box.primitive_poses[0].position.z;

    std::cout << box.primitive_poses[0].orientation << std::endl;
       

    if (dx<0.8||dy<0.8)
    {
        // Calculate the X dimension
        if (dx<0.8)
        {
            double sx = dx/2; // point saphe in x
            for (double i = p.x+sx; i < p.x+sx+range; i= i + 0.02)
            {
                for (double j = p.z-(dz/2); j < p.z+(dz/2); j= j + 0.02)
                {
                    geometry_msgs::Pose lp;
                    lp.position.x = i;
                    lp.position.y = p.y;
                    lp.position.z = j;

                    list.push_back(lp);
                }
                
            }
            
            for (double i = p.x-sx-range; i < p.x-sx; i= i + 0.02)
            {
                for (double j = p.z-(dz/2); j < p.z+(dz/2); j= j + 0.02)
                {
                    geometry_msgs::Pose lp;
                    lp.position.x = i;
                    lp.position.y = p.y;
                    lp.position.z = j;

                    list.push_back(lp);
                }
            }
        }
        // Calculate the Y dimension
        if (box.primitives[0].dimensions[1]<0.8)
        {
            double sy = dy/2; // point saphe in x
            for (double i = p.y+sy; i < p.y+sy+range; i= i + 0.02)
            {
                for (double j = p.z-(dz/2); j < p.z+(dz/2); j= j + 0.02)
                {
                    geometry_msgs::Pose lp;
                    lp.position.x = p.x;
                    lp.position.y = i;
                    lp.position.z = j;
              
                    list.push_back(lp);
                }
                
            }
            
            for (double i = p.y-sy-range; i < p.y-sy; i= i + 0.02)
            {
                for (double j = p.z-(dz/2); j < p.z+(dz/2); j= j + 0.02)
                {
                    geometry_msgs::Pose lp;
                    lp.position.x = p.x;
                    lp.position.y = i;
                    lp.position.z = j;
               
                    list.push_back(lp);
                }
                
            }        

        }

        
        
        
    }else
    {
        std::cout << "Out of the Gripper grasping range" << std::endl;
    }
    



}

void cilinder(moveit_msgs::CollisionObject box)
{
    std::cout << "Recognition cilinder" << std::endl;
    
    double dz = box.primitives[0].dimensions[0];
    double r = box.primitives[0].dimensions[1];


    geometry_msgs::Point p;
    p.x = box.primitive_poses[0].position.x;
    p.y = box.primitive_poses[0].position.y;
    p.z = box.primitive_poses[0].position.z;

    if (r<0.4)
    {

        for(double angle=M_PI/2; angle<=(2*M_PI)+(M_PI/2); angle+=0.2)
        {
            for (double i = 0.04; i < range; i=i+0.01)
            {
                double PX = p.x + (r+i)*cos( angle ); 
                double PY = p.y + (r+i)*sin( angle );
                // Point( Center.x + radius*cos( angle ), Center.y + radius*sin( angle ) );
                for (double j = p.z-(dz/2); j < p.z+(dz/2); j= j + 0.02)
                {
                    geometry_msgs::Pose lp;
                    lp.position.x = PX;
                    lp.position.y = PY;
                    lp.position.z = j;
                    lp.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,-M_PI/2,angle-(M_PI/2));
                    // 0,0,range-M_PI
                    list.push_back(lp);
                }
            }
        }

    }else
    {
        std::cout << "Out of the Gripper grasping range" << std::endl;
    }
}

void sphere(moveit_msgs::CollisionObject box)
{
    std::cout << "Recognition sphere" << std::endl;
    
    double r = box.primitives[0].dimensions[0];

    geometry_msgs::Point p;
    p.x = box.primitive_poses[0].position.x;
    p.y = box.primitive_poses[0].position.y;
    p.z = box.primitive_poses[0].position.z;

    if (r<0.4)
    {

        for(double i = 0.005; i < range; i=i+0.01)
        {
            for (double alpha=0; alpha<=2*M_PI; alpha+=0.5)
            {
                for (double beta=0; beta<=M_PI; beta+=0.5)
                {
                    double PX = p.x + (r+i)*cos( alpha )*sin( beta ); 
                    double PY = p.y + (r+i)*sin( alpha )*sin( beta );
                    double PZ = p.z + (r+i)*cos(beta);

                    geometry_msgs::Pose lp;
                    lp.position.x = PX;
                    lp.position.y = PY;
                    lp.position.z = PZ;
                    lp.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,alpha-M_PI,beta);
                    list.push_back(lp);
                }
            }
                
                
        }

    }else
    {
        std::cout << "Out of the Gripper grasping range" << std::endl;
    }
}


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


bool getIK_right(geometry_msgs::Pose eef_pose_right)
{
  kinematic_state->enforceBounds();

  Eigen::Isometry3d pose_right;
  tf::poseMsgToEigen(eef_pose_right, pose_right);
  // cout << "Pose: " << eef_pose << "\n";
  
  bool found_ik =kinematic_state->setFromIK(group_right, pose_right, 10, 0.5);

  return found_ik;

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

  group_right = kinematic_model->getJointModelGroup("right_eef");
  group_right->getJointModelNames();
  
  group_left = kinematic_model->getJointModelGroup("left_arm");
  group_left->getJointModelNames();

  reset_joint_values();

}



void chatterCallback(const moveit_msgs::CollisionObject& msg)
{
    // list.clear();
    
  
    if (msg.primitives[0].type==msg.primitives[0].BOX)
    {
        ROS_INFO("Is BOX");
        box(msg);
    }else if (msg.primitives[0].type==msg.primitives[0].SPHERE)
    {
        ROS_INFO("Is SPHERE");
        sphere(msg);
    }else if (msg.primitives[0].type==msg.primitives[0].CYLINDER)
    {
        ROS_INFO("Is CILINDER");
        cilinder(msg);
    }else
    {
        ROS_INFO("NOT RECOGNITION");
        
    }

    std::cout << list.size() << std::endl;




    // Clear messages
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
    for (int i = 0; i < list.size(); i++)
    {
        std::cout << "N: " << i <<std::endl;
        // if (getIK_right(list.at(i)))
        // {
            // Retrieve joint values map
            std::map<std::string, double> goal_joint_values;
            std::vector<double> joint_values;
            kinematic_state->copyJointGroupPositions(group_left, joint_values);
            const std::vector<std::string> &iiwa_joint_names = group_left->getJointModelNames();
            for(std::size_t i = 0; i < iiwa_joint_names.size(); ++i) 
            { // TODO cleanly remove the fixed joints (first and last) from the list
            goal_joint_values[iiwa_joint_names[i]] = joint_values[i];
            // cout << iiwa_joint_names[i] << " = " << joint_values[i] << "\n";
            }

            const Eigen::Affine3d &eef_state = kinematic_state->getGlobalLinkTransform("right_end_effect_point");
            geometry_msgs::Pose eef_pose, eef_final;
            tf::poseEigenToMsg(eef_state, eef_pose);
            tf::poseEigenToMsg(eef_state, eef_final);
            eef_final.position.x=eef_final.position.x+0.05;
            eef_final.position.y=eef_final.position.y+0.05;
            eef_final.position.z=eef_final.position.z+0.05;
            // cout << eef_pose << "\n";

            double w= manipulability_right();
            std::cout << w << "\n";

            // std::cout << list.at(i) << std::endl;
            // drawing points
            geometry_msgs::Vector3 scale = visual_tools->getScale(rviz_visual_tools::SMALL);
            std_msgs::ColorRGBA color = visual_tools->getColorScale(w*10);
            visual_tools->publishSphere(visual_tools->convertPose(list.at(i)), color, scale, "Sphere");
            // visual_tools->publishZArrow(visual_tools->convertPose(eef_final), rviz_visual_tools::ORANGE, rviz_visual_tools::XXXXSMALL);
            // visual_tools->publishLine(visual_tools->convertPose(eef_pose), visual_tools->convertPose(eef_final), rviz_visual_tools::RAND);
            visual_tools->publishAxis(visual_tools->convertPose(list.at(i)), rviz_visual_tools::XXXXSMALL);
            visual_tools->trigger();
        
        // }else
        // {
        //     std::cout << "IK not found" << std::endl;
        // }
        
    }

    return;
  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_off_pose");
    ros::NodeHandle n;
    node_handle = &n;
   
    initialize();

    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_footprint","/visualization_marker_array"));
    visual_tools->loadMarkerPub();

    ros::Subscriber sub = n.subscribe("object",1000, chatterCallback);

    std::cout << "ver los circulos" << std::endl;





    // ros::spinOnce();
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}