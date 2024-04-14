#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher vel_publisher;
double current_x,current_y;
// Define goal tolerance for reaching the target (adjust as needed)
const double GOAL_TOLERANCE = 0.1; // meters (minimum distance considered "reached")

// Define constants for smoother movements
const double MAX_LINEAR_VEL = 0.2; // m/s (adjust based on your TurtleBot's capabilities)
const double MAX_ANGULAR_VEL = 0.5; // rad/s (adjust based on your desired turning speed)
const double ACCELERATION_STEP = 0.01; // m/s^2 or rad/s^2 (adjust for smoother acceleration)

double current_linear_vel = 0.0;
double current_angular_vel = 0.0;
double goal_x, goal_y; // Variables to store the received goal coordinates

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Extract current robot position from odometry data
  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Extract goal coordinates from the message
  goal_x = msg->pose.position.x;
  goal_y = msg->pose.position.y;
}

template<typename T>
T clamp11(const T& value, const T& low, const T& high) {
  if (value < low) {
    return low;
  } else if (value > high) {
    return high;
  } else {
    return value;
  }
}

void velocityControl() {
  // Calculate distance to the goal
  double distance_to_goal = sqrt(pow(goal_x - current_x, 2) + pow(goal_y - current_y, 2));

  // Determine desired linear velocity based on distance to goal
  double desired_linear_vel = std::min(distance_to_goal, MAX_LINEAR_VEL);

  // Calculate the angle needed to reach the goal
  double angle_to_goal = atan2(goal_y - current_y, goal_x - current_x);

  // Calculate required angular velocity for smooth turning
  double angular_error = angle_to_goal - current_angular_vel; // Consider odometry for more accurate angle

  // Apply proportional control for angular velocity with saturation
  double desired_angular_vel = std::min(MAX_ANGULAR_VEL * std::tanh(angular_error), MAX_ANGULAR_VEL);

  // Implement smooth acceleration for both linear and angular velocities
  current_linear_vel = std::min(current_linear_vel + ACCELERATION_STEP, desired_linear_vel);
  current_angular_vel = std::min(current_angular_vel + ACCELERATION_STEP, desired_angular_vel);

  // Ensure velocities stay within safe limits
  current_linear_vel = clamp11(current_linear_vel, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  current_angular_vel = clamp11(current_angular_vel, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  // Publish the smoothed velocities
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = current_linear_vel;
  twist_msg.angular.z = current_angular_vel;
  vel_publisher.publish(twist_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "turtlebot_goal_controller");
  ros::NodeHandle nh;

  // Create a publisher for the cmd_vel topic
vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Subscribe to odometry topic to get current robot position
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

  // Subscribe to the goal topic (replace with your actual topic name)
  ros::Subscriber goal_sub = nh.subscribe("/local_goal", 1, goalCallback);

  ros::Rate loop_rate(10); // Control loop rate (adjust as needed)

  while (ros::ok()) {
    velocityControl(); // Call the velocity control function in the main loop

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}