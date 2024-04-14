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
#include "d_star_lite.cpp"

using namespace std;


double resolution = 0.0;
double start_x,start_y,yaw,goal_x,goal_y;
bool got_start_coords = false, got_goal_coords = false;
vector<float> src1(2);
ros::Publisher path_pub;
ros::Publisher vel_pub;


double calc_euclidean(vector<float> src, vector<float> dest)
{
	// Return using the distance formula
	return ((double)sqrt(
		pow((src[0] - dest[0]),2.0) 
		+ pow((src[1] - dest[1]),2.0)));
}

float steering_angle(vector<float> src , vector<float> dest)
{
    return atan2((dest[1] - dest[1]),(src[0] - src[1]));
}


float linear_vel(vector<float> src , vector<float> dest, float constant = 0.5)
{
    return (constant * calc_euclidean(src,dest));
}
float angular_vel(vector<float> src , vector<float> dest, float constant = 0.15)
{
	float ang = steering_angle(src,dest);
	if(fabs(ang-yaw) < 1.57)
		return 0;
	// cout<<"Steering Angle"<<ang<<" ";
	if((ang - yaw)  >= 1.57)
		return (fabs(ang-yaw) * constant);
	else if((ang - yaw) <= -1.57)
		return -(constant * fabs(ang-yaw));
	else
		return 0;

}


void travel(vector<float> src,vector<float> goal)
{
	// src[0] = src[1];
	// src[1] = src1[1];

	geometry_msgs::Twist vel_msg;
	if(calc_euclidean(src,goal) > 0.1)
	{
		cout<<"Distance from the goal:"<<calc_euclidean(src,goal)<<" Goal:"<<goal[0]<<" Source:"<<src[0]<<" Yaw:"<<yaw<<endl;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		
		float ang_vel = angular_vel(src,goal);
		vel_msg.angular.z = ang_vel;

		if(ang_vel == 0)
			vel_msg.linear.x = linear_vel(src,goal);
		else
			vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		

		vel_pub.publish(vel_msg);
	}

    // if (turtlebot::dir == 0) {
    //     vel_msg.linear.x = 0.1;
    //     vel_msg.angular.z = 0.15;
    //     vel_pub.publish(vel_msg);
    //     rate.sleep();
    //     ROS_INFO_STREAM("Turning Left");
    // }
    // // If direction is straight
    // if (turtlebot::dir == 1) {
    //     vel_msg.linear.x = 0.15;
    //     vel_msg.angular.z = 0;
    //     vel_pub.publish(vel_msg);
    //     rate.sleep();
    //     ROS_INFO_STREAM("Straight");
    // }
    // // If direction is right
    // if (turtlebot::dir == 2) {
    //     vel_msg.linear.x = 0.1;
    //     vel_msg.angular.z = -0.15;
    //     vel_pub.publish(vel_msg);
    //     rate.sleep();
    //     ROS_INFO_STREAM("Turning Right");
    // }
    // // If robot has to search
    // if (turtlebot::dir == 3) {
    //     vel_msg.linear.x = 0;
    //     vel_msg.angular.z = 0.25;
    //     vel_pub.publish(vel_msg);
    //     rate.sleep();
    //     ROS_INFO_STREAM("Searching");
    //}   
	else
	{
	cout<<"Reached the Required Node"<<endl;
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	vel_pub.publish(vel_msg);

	}
}


void get_yaw(const nav_msgs::Odometry::ConstPtr& pos)
{
	
	auto x = pos->pose.pose.orientation.x;
	auto y = pos->pose.pose.orientation.y;
	auto z = pos->pose.pose.orientation.z;
	auto w = pos->pose.pose.orientation.w;
	tf::Quaternion quat;
	quat.setX(x);
	quat.setY(y);
	quat.setZ(z);
	quat.setW(w);
	tf::Matrix3x3 m(quat);
	double roll, pitch;
	m.getEulerYPR(yaw,pitch,roll);
	
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& pos)
{
    if(!got_start_coords)
    {
      got_start_coords = true;
      start_x = pos->pose.pose.position.x;
      start_y = pos->pose.pose.position.y;
    }
    get_yaw(pos);

    src1[0] = pos->pose.pose.position.x;
    src1[1] = pos->pose.pose.position.y;
    vector<float> goal(2);
    goal[0] = 1.38;
    goal[1] = -0.744;
    //travel(goal);

}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    goal_x = pose->pose.position.x;
    goal_y = pose->pose.position.y;
    got_goal_coords = true;
}

// void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
// {   
//     ROS_INFO("In PoseCallback");
//     start_x = pose->pose.position.x;
//     start_y = pose->pose.position.y;
//     got_start_coords = true;
// }


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) 
{
    cout<<"Map initialized"<<endl;
    int width = map->info.width;
    int height = map->info.height;
    resolution = map->info.resolution;
    // ROW = width;
    // COL = height;
    std::vector<std::vector<int>> grid(width, std::vector<int>(height, 0));
	  int index =0;
    // Convert the occupancy grid to a 2D vector
    if(got_goal_coords)
    {
    for (int x = 0; x < map->info.width; ++x) {
        for (int y = 0; y < map->info.height; ++y) {
            int index = y * map->info.width + x;
            int value= map->data[index];
			
            if(value == 100)
                grid[x][y] = 1;
            else
                grid[x][y] = 0;
        }
    }
    int src_x =(int)((-1.93+10)/resolution );
    int src_y = (int)((-0.462+10)/resolution );
    int dest_x = (int)((goal_x+10)/resolution );
    int dest_y = (int)((goal_y+10)/resolution );
    cout<<src_x<<src_y<<dest_x<<dest_y<<endl;
    Node start(src_x, src_y, 0, 0, 0, 0);
    Node goal(dest_x,dest_y, 0, 0, 0, 0);
    constexpr int n = 21;
    start.id_ = start.x_ * n + start.y_;
    start.pid_ = start.x_ * n + start.y_;
    goal.id_ = goal.x_ * n + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
    // Make sure start and goal are not obstacles and their ids are correctly
    // assigned.
    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;


    // Store points after algorithm has run
    std::vector<std::vector<int>> main_grid = grid;

    grid = main_grid;
    DStarLite d_star_lite(grid);

    auto [path_found,path_vector] = d_star_lite.Plan(start,goal);
    if(path_found)
    {
        for(Node p:path_vector)
        {
        cout<<p.x_<<" "<<p.y_<<endl;
        }
    }
    

        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time(0);
        path_msg.header.frame_id = "map"; 
        int count = 0;
        // Populate the path message with poses
        cout<<path_found<<" "<<path_vector.size()<<endl;
        for(Node p:path_vector)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time(0);
            pose_stamped.header.frame_id = "map"; 
            pose_stamped.pose.position.x = p.x_*resolution - 10.0;
            pose_stamped.pose.position.y = p.y_*resolution - 10.0;
            pose_stamped.pose.position.z = 0; 
            pose_stamped.pose.orientation.w = 1; 

            // Add the pose to the path
            path_msg.poses.push_back(pose_stamped);
        }
        cout<<"Publishing Path"<<endl;
        vector<float> source = {(float)src_x,(float)src_y};
        for(Node p:path_vector)
        {
            vector<float> g1 = {(float)p.x_,(float)p.y_};
            //while(src1[0] != g1[0] && src1[1]!=g1[1])
            travel(source,g1);

            source  = g1;
        }
        while(ros::ok())
        path_pub.publish(path_msg);
        got_goal_coords = false;
    }
    else
    ROS_INFO("Goal Coordinates not obtained yet");
}







int main(int argc, char** argv) 
{
  ros::init(argc, argv, "TH3plan");
  ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(1);
  
  //ros::Rate rate(10);
  //ros::Subscriber pos = nh.subscribe("odom",1000,odomCallback);
  ros::Subscriber sub = nh.subscribe("map", 1000 ,mapCallback);
  //ros::Subscriber pose_sub = nh.subscribe("initialpose",1000,poseCallback);
  ros::Subscriber goal_sub = nh.subscribe("move_base_simple/goal",1000,goalCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom",1000,goalCallback);
  path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  //ros::Publisher pub = nh.advertise<std_msgs::String>("say", 1000);
  ros::spin();

  //rate.sleep();
  return 0;
}
