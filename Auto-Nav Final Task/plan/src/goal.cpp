#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <iostream>
#include <random>

geometry_msgs::PoseStamped goal;
ros::Publisher goal_pub;

void goalCallback(const nav_msgs::Path &path)
{

    ROS_INFO("PUblishing Goal on the 'local_goal' Topic");
    geometry_msgs::PoseStamped pose;
    ros::Rate rate(0.5);
    for(geometry_msgs::PoseStamped p : path.poses)
    {
        pose = p;
        goal_pub.publish(pose);
        rate.sleep();
    }
    while(ros::ok())
    {  
    //ros::spinOnce();
    goal_pub.publish(pose);
    }

}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ThePlanPart2");
  ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(1);
  
  //ros::Rate rate(10);
  //ros::Subscriber pos = nh.subscribe("odom",1000,odomCallback);
  //ros::Subscriber pose_sub = nh.subscribe("initialpose",1000,poseCallback);

  ros::Subscriber goal_sub = nh.subscribe("/path",1000,goalCallback);
  goal_pub =  nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1000);


  ros::spin(); 
  //rate.sleep();
  return 0;
}
