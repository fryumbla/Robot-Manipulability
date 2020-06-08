#include "ros/ros.h"

// estas librerias es para los objetos moveit_msgs::CollisionObject t de moveit
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometric_shapes/shape_operations.h"


#include <math.h>

// esta es para vizualizar en rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/tf.h>


#include <geometry_msgs/Point.h>

ros::NodeHandle *node_handle = NULL;


std::vector<geometry_msgs::Pose> list;
// For visualizing things in rviz

rviz_visual_tools::RvizVisualToolsPtr visual_tools;

double range = 0.05;

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

        for(double angle=0.0; angle<=2*M_PI; angle+=0.3)
        {
            for (double i = 0.005; i < range; i=i+0.01)
            {
                double PX = p.x + (r+i)*cos( angle ); 
                double PY = p.y + (r+i)*sin( angle );
                // Point( Center.x + radius*cos( angle ), Center.y + radius*sin( angle ) );
                for (double j = p.z-(dz/2); j < p.z+(dz/2); j= j + 0.01)
                {
                    geometry_msgs::Pose lp;
                    lp.position.x = PX;
                    lp.position.y = PY;
                    lp.position.z = j;
                    lp.orientation= tf::createQuaternionMsgFromRollPitchYaw(0,0,angle-M_PI);
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
        // std::cout << list.at(i) << std::endl;
        // drawing points
        geometry_msgs::Vector3 scale = visual_tools->getScale(rviz_visual_tools::SMALL);
        std_msgs::ColorRGBA color = visual_tools->getColorScale(0.01*10);
        visual_tools->publishSphere(visual_tools->convertPose(list.at(i)), color, scale, "Sphere");
        // visual_tools->publishZArrow(visual_tools->convertPose(eef_final), rviz_visual_tools::ORANGE, rviz_visual_tools::XXXXSMALL);
        // visual_tools->publishLine(visual_tools->convertPose(eef_pose), visual_tools->convertPose(eef_final), rviz_visual_tools::RAND);
        visual_tools->publishAxis(visual_tools->convertPose(list.at(i)), rviz_visual_tools::XXXXSMALL);
        visual_tools->trigger();
    }

    return;
  
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_off_pose");
	ros::NodeHandle n;
	node_handle = &n;
    
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_link","/visualization_marker_array"));
    visual_tools->loadMarkerPub();

    ros::Subscriber sub = n.subscribe("object",1000, chatterCallback);

    std::cout << "ver los circulos" << std::endl;





    // ros::spinOnce();
    // ros::waitForShutdown();
    ros::spin();

    return 0;
}