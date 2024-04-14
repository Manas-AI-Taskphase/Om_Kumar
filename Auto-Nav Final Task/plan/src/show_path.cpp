#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"

ros::Subscriber path_sub;
ros::Publisher path_pub;

void pathCallback(nav_msgs::Path path)
{
    while(ros::ok())
    {
    path_pub.publish(path);
    ros::spinOnce();
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "LightItUp");

    ros::NodeHandle nh;

    path_sub = nh.subscribe("local_path",1000,pathCallback);
    path_pub = nh.advertise<nav_msgs::Path>("path",1000);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path",1000);
    ros::spin();
    
}