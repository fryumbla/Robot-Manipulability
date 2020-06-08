#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// #include <rviz_visual_tools/rviz_visual_tools.h>

#include <iostream> 
#include <vector> 
  
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  sleep(2.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface group("arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //	
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  vector<double> a=group.getCurrentJointValues();
  //cout << a.at(0) << std::endl;
  group.setPlanningTime(1);//0.1
  
  //group.setGoalTolerance(0.5);
  std::vector< geometry_msgs::Pose > poses;

  // geometry_msgs::Pose target_pose3 = group.getCurrentPose().pose;
  // std::vector<geometry_msgs::Pose> waypoints;

  for(float i=5;i<=18; ++i)
  {
   

    cout << i << std::endl;
    geometry_msgs::Pose home_pose1;
    home_pose1.orientation.w = 2.87694e-05;
    home_pose1.orientation.x= 3.29237e-05;
    home_pose1.orientation.y = 3.29132e-09;
    home_pose1.orientation.z = 1;

    home_pose1.position.x = 1.91352e-05;
    home_pose1.position.y = 0.00526172;
    home_pose1.position.z = 0.155+((3*i)/100);//755
    cout << home_pose1.position.z << std::endl;
    
    // poses.push_back(home_pose1);
    group.setPoseTarget(home_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan); 
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success.val ? "":"FAILED");    

    // Sleep to give Rviz time to visualize the plan. 
    group.move();	
  // END_TUTORIAL





  }

  // group.setMaxVelocityScalingFactor(0.1);

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
  
  // my_plan.trajectory_=trajectory;
  // group.execute(my_plan);



  // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.



  





  ros::shutdown();  

 return 0;
}
