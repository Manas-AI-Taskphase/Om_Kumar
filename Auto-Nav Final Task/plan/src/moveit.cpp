#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

ros::Subscriber path_sub;
ros::Publisher move_it;

void pathCallback(nav_msgs::Path path)
{

    for(geometry_msgs::PoseStamped p:path.poses)
    {
   
       move_it.publish(p);
       ros::spinOnce();
    }
    ROS_INFO("Gave all the points");
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    while(ros::ok())
    move_it.publish(pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TheMover");

    ros::NodeHandle nh;
    //ros::Subscriber pose_sub = nh.subscribe("move_base_simple/goal",1000,poseCallback);
    path_sub = nh.subscribe("path",1000,pathCallback);
    move_it = nh.advertise<geometry_msgs::PoseStamped>("local_goal",1000);
    ros::spin();
    
}
