#include <ros/ros.h>
#include "depth_footstep_planner/FootstepPlannerNode.h"

//void callback(const depthmap_humanoid_msgs::DepthMapConstPtr& msg){
//  double d;
//  depthmap2d::DepthMap2D map(msg);
//  depth_footstep_planner::collision_check(250,500,0,d,32,20,0,map);

//}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_footstep_planner");
//  ros::NodeHandle nh;
//  ros::Subscriber s = nh.subscribe<depthmap_humanoid_msgs::DepthMap>("/depthmap",1,&callback);
//  depth_footstep_planner::euclidean_distance_sq(1,2,3,4);
  depth_footstep_planner::FootstepPlannerNode planner;
  ROS_INFO("Hello world!");
  ros::spin();
}
