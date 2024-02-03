#include <ros/ros.h>
#include "depth_footstep_planner/FootstepPlannerNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_footstep_planner_node");
  depth_footstep_planner::FootstepPlannerNode planner;

  ROS_INFO("Depth Footstep Planner Node Started");
  ros::spin();
}
