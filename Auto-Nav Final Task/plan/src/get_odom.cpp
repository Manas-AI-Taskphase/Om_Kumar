#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"


using namespace std;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;
ros::Subscriber map_sub;

ros::Publisher star_start;
ros::Publisher star_goal;
ros::Publisher star_map;

nav_msgs::OccupancyGrid grid;

double obs_threshold = 1.0;
bool get_path = true, got_map = false, sent_map = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    auto pos = msg->pose.pose.position;
    if(get_path && got_map && sent_map)
    {   
        get_path = false;
        sent_map = false;
        star_start.publish(msg);
    }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &data)
{
    vector<float> range = data->ranges;
    if(range[0] < obs_threshold || range[45]< obs_threshold || range[325]< obs_threshold)
    {   
        ROS_INFO("Detected Obstacle, Remapping");
        get_path = true;
    }
    
}

void mapCallback(const nav_msgs::OccupancyGrid map)
{
    if(!sent_map && get_path)
    {
        star_map.publish(map);
        got_map = true;
        sent_map = true;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "TheGuide");

    ros::NodeHandle nh;

    odom_sub = nh.subscribe("/odom",1000,odomCallback);
    laser_sub = nh.subscribe("/scan",1000,laserCallback);
    map_sub = nh.subscribe("/map",1000,mapCallback);
    star_start = nh.advertise<nav_msgs::Odometry>("/gimme_src",1000);
    star_map = nh.advertise<nav_msgs::OccupancyGrid>("look",1000);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path",1000);
    

    ros::spin();
    return 0;

    
}