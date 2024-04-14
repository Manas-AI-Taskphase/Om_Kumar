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
#include <std_msgs/Bool.h>

geometry_msgs::PoseStamped goal;
ros::Publisher map_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    ROS_INFO("PUblishing Map on the 'local_map' Topic");
    while(ros::ok())
    {  
    ros::spinOnce();
    map_pub.publish(map);
    }
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ThePlanPart3");
  ros::NodeHandle nh;

  ros::Subscriber goal_sub = nh.subscribe("map",1000,mapCallback);
  map_pub =  nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1000);
  ros::spin(); 

  return 0;
}
