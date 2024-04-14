//#include "d_star_lite.hpp"

#include "planner.hpp"
#include "utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <random>
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

using namespace std;
ros::Subscriber map_sub;
ros::Publisher goal_pub;
ros::Subscriber goal_sub;
vector<vector<int>> grid_global;
double resolution;Node goal;Node start;
bool got_goal_coords = false, got_map = false;
constexpr int pause_time = 500;  // milliseconds


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  int width = map->info.width;
  int height = map->info.height;
  resolution = map->info.resolution;
  vector<vector<int>> grid(width,(vector<int>(height)));
  int index = 0;
  for(int i=0;i<width;i++)
  {
    for(int j=0;j<height;j++)
    {
      grid[i][j] = map->data[index];
      index++;
    }
  }
  grid_global = grid;
  got_map = true;
}

void send_goal(Node dest)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time(0);
  pose_stamped.header.frame_id = "map"; 
  pose_stamped.pose.position.x = dest.x_*resolution - 10.0;
  pose_stamped.pose.position.y = dest.y_*resolution - 10.0;
  pose_stamped.pose.position.z = 0; 
  pose_stamped.pose.orientation.w = 1; 

  goal_pub.publish(pose_stamped);
}


void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{ 

    goal.x_ = pose->pose.position.x;
    goal.y_ = pose->pose.position.y;  
    cout<<"Goal is "<<pose->pose.position.x;
    got_goal_coords = true;
}

class DStarLite:public Planner
{
  public:
  std::vector<std::vector<double>> CreateGrid();

  std::vector<std::vector<double>> rhs_;
  std::vector<std::vector<double>> g_;
  std::unordered_map<int, std::vector<Node>> time_discovered_obstacles_{};
  std::vector<Node> motions_{};
  LazyPQ U_;
  Node start_, goal_, last_;
  double k_m_ = 0;
  Key k_old_{0, 0};
  int time_step_ = 0;
  bool create_random_obstacles_ = true;

DStarLite(std::vector<std::vector<int>> grid)
      : Planner(std::move(grid)) {}


bool IsObstacle(const Node& n) const {
  return grid_[n.x_][n.y_] == 1;
}

double H(const Node& n1, const Node& n2) const {
  return std::sqrt(std::pow(n1.x_ - n2.x_, 2) + std::pow(n1.y_ - n2.y_, 2));
}

std::vector<Node> GetNeighbours(const Node& u) const {
  std::vector<Node> neighbours;
  for (const auto& m : motions_) {
    if (const auto neighbour = u + m;
        !checkOutsideBoundary(neighbour, grid_.size())) {
      neighbours.push_back(neighbour);
    }
  }
  return neighbours;
}

std::vector<Node> GetPred(const Node& u) const {
  return GetNeighbours(u);
}

std::vector<Node> GetSucc(const Node& u) const {
  return GetNeighbours(u);
}

double C(const Node& s1, const Node& s2) const {
  if (IsObstacle(s1) || IsObstacle(s2)) {
    return std::numeric_limits<double>::max();
  }
  const Node delta{s2.x_ - s1.x_, s2.y_ - s1.y_};
  return std::find_if(std::begin(motions_), std::end(motions_),
                      [&delta](const Node& motion) {
                        return CompareCoordinates(motion, delta);
                      })
      ->cost_;
}

Key CalculateKey(const Node& s) const {
  return Key{std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_]) + H(start_, s) + k_m_,
             std::min(g_[s.x_][s.y_], rhs_[s.x_][s.y_])};
}

std::vector<std::vector<double>> CreateDummyGrid() {
   return std::vector<std::vector<double>>(
      grid_global.size(), std::vector<double>(grid_global[0].size(), std::numeric_limits<double>::max()));
}

void Initialize() {
  motions_ = GetMotion();
  time_step_ = 0;
  U_.clear();
  k_m_ = 0;
  rhs_ = CreateDummyGrid();
  g_ = CreateDummyGrid();
  rhs_[goal_.x_][goal_.y_] = 0;
  U_.insert(NodeKeyPair{goal_, CalculateKey(goal_)});
}

void UpdateVertex(const Node& u) {
  if (grid_[u.x_][u.y_] == 0) {
    grid_[u.x_][u.y_] = 2;
  }
  if (!CompareCoordinates(u, goal_)) {
    rhs_[u.x_][u.y_] = std::numeric_limits<double>::max();
    const auto successors = GetSucc(u);
    for (const auto& sprime : successors) {
      rhs_[u.x_][u.y_] =
          std::min(rhs_[u.x_][u.y_], C(u, sprime) + g_[sprime.x_][sprime.y_]);
    }
  }
  if (U_.isElementInStruct({u, {}})) {
    U_.remove(NodeKeyPair{u, Key()});
  }
  if (rhs_[u.x_][u.y_] != g_[u.x_][u.y_]) {
    U_.insert(NodeKeyPair{u, CalculateKey(u)});
  }
}

void ComputeShortestPath() {
  while ((!U_.empty() && U_.top().key < CalculateKey(start_)) ||
         (rhs_[start_.x_][start_.y_] != g_[start_.x_][start_.y_])) {
    k_old_ = U_.top().key;
    const Node u = U_.top().node;
    U_.pop();
    if (const Key u_key = CalculateKey(u); k_old_ < u_key) {
      U_.insert(NodeKeyPair{u, u_key});
    } else if (g_[u.x_][u.y_] > rhs_[u.x_][u.y_]) {
      g_[u.x_][u.y_] = rhs_[u.x_][u.y_];
      for (const auto& s : GetPred(u)) {
        UpdateVertex(s);
      }
    } else {
      g_[u.x_][u.y_] = std::numeric_limits<double>::max();
      for (const auto& s : GetPred(u)) {
        UpdateVertex(s);
      }
      UpdateVertex(u);
    }
  }
}

std::vector<Node> DetectChanges() 
{
  std::vector<Node> obstacles;
  std::vector<vector<int>> old_grid = grid_global;
  ros::spinOnce();
  for(int x=0;x<grid_global.size();x++)
  {
      for(int y=0;y<grid_global[0].size();y++)
      {
        if((grid_global[x][y] != old_grid[x][y] && grid_global[x][y]==1))
            obstacles.emplace_back(Node(x,y,0,0,0,0));
      }
    
  }
  return obstacles;
}

std::tuple<bool, std::vector<Node>> Plan(const Node& start,const Node& goal)
{
  grid_ = grid_global;
  start_ = start;
  goal_ = goal;
  std::vector<Node> path;
  path.push_back(start_);
  grid_[start_.x_][start_.y_] = 4;
  //PrintGrid(grid_);
  auto last = start_;
  Initialize();
  ComputeShortestPath();
  while (!CompareCoordinates(start_, goal_)) 
  {
    time_step_++;
    if (g_[start_.x_][start_.y_] == std::numeric_limits<double>::max()) 
    {
      path.clear();
      path.push_back(start);
      path.back().cost_ = -1;
      std::cout << "The path has been blocked" << '\n';
      return {false, path};
    }
    const auto successors = GetSucc(start_);

    grid_[start_.x_][start_.y_] = 3;
    start_ = *std::min_element(std::begin(successors), std::end(successors),
                               [this](const auto& n1, const auto& n2) {
                                 return C(start_, n1) + g_[n1.x_][n1.y_] <
                                        C(start_, n2) + g_[n2.x_][n2.y_];
                               });
    send_goal(start_);
    path.push_back(start_);
    grid_[start_.x_][start_.y_] = 4;

    const auto changed_nodes = DetectChanges(); 
    if (!changed_nodes.empty()) 
    {
      k_m_ += H(last, start_);
      last = start;
      
      for (const auto node : changed_nodes) 
        UpdateVertex(node);
      
      ComputeShortestPath();
    }

  }
  path[0].id_ = path[0].x_ * n_ + path[0].y_;
  path[0].pid_ = path[0].id_;
  path[0].cost_ = 0;
  for (int i = 1; i < path.size(); i++) 
  {
    path[i].id_ = path[i].x_ * n_ + path[i].y_;
    const auto delta =
        Node(path[i].x_ - path[i - 1].x_, path[i].y_ - path[i - 1].y_);
    path[i].cost_ = path[i - 1].cost_ +
                    std::find_if(std::begin(motions_), std::end(motions_),
                                 [&delta](const Node& motion) {
                                   return CompareCoordinates(motion, delta);
                                 })
                        ->cost_;
    path[i].pid_ = path[i - 1].id_;
  }
  return {true, path};
  }


};


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ThePlanPart4");
  ros::NodeHandle nh;

  map_sub = nh.subscribe("/local_map",1000,mapCallback);
  goal_sub = nh.subscribe("move_base_simple/goal",1000,goalCallback);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1000);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path",1000);
  ros::Rate rate(3);
  while(ros::ok())
  {
    ROS_INFO("Waiting for Map");
    ros::spinOnce();
    rate.sleep();
    if(got_map)
      break;
  }


  while(ros::ok())
  {
    ROS_INFO("Waiting for Goal");
    ros::spinOnce();
    rate.sleep();
    if(got_goal_coords)
      break;
  }


  int n = 21; //Any number to generate the ID 

  goal.x_ = (int)((goal.x_+10)/resolution );
  goal.y_ = (int)((goal.y_+10)/resolution );

  start.x_ = (int)((-1.93+10)/resolution );
  start.y_ = (int)((-0.462+10)/resolution );

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

  if(grid_global[goal.x_][goal.y_] == 1)
  {
    ROS_INFO("The goal is invalid..Exiting");
    exit(0);
  }
  DStarLite d_star_lite(grid_global);

  const auto [found_path, path_vector] = d_star_lite.Plan(start, goal);
  if(found_path)
  {
    ROS_INFO("Found the Path, Publishing it");
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time(0);
    path_msg.header.frame_id = "map"; 
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
    while(ros::ok())
    path_pub.publish(path_msg);
  }
  return 0;
}


