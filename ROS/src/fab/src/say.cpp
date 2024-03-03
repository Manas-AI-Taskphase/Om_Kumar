#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <sstream>

#define MAX_MSG_LEN 1000
using namespace std;
void callback(const std_msgs::String& msg) {
  ROS_INFO(msg.data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_recieve");
  ros::NodeHandle nh;

  //ros::MultiThreadedSpinner spinner(2); // Use 4 threads
  ros::AsyncSpinner spinner(2);
  
  
  ros::Subscriber sub = nh.subscribe("talk", 1000 ,callback);
  ros::Publisher pub = nh.advertise<std_msgs::String>("say", 1000);
  std_msgs::String msg;
  msg.data = "Hello";
  

  ROS_INFO("User A is LIve!");
  spinner.start();
  ros::Rate rate(1);
  while(ros::ok()) {
    
    char inp[MAX_MSG_LEN];
    cin.getline(inp,MAX_MSG_LEN);
    
    msg.data = inp;
    pub.publish(msg);
    
  
    //ros::spinOnce();
    //rate.sleep();
  }

  return 0;
}
