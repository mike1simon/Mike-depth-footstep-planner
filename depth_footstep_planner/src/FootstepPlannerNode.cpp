
#include <depth_footstep_planner/FootstepPlannerNode.h>

namespace depth_footstep_planner
{
FootstepPlannerNode::FootstepPlannerNode()
{
  ros::NodeHandle nh;

  // provide callbacks to interact with the footstep planner:
  ivGridMapSub = nh.subscribe<depthmap_humanoid_msgs::DepthMap>("depthmap", 1, &FootstepPlanner::mapCallback, &ivFootstepPlanner);
  ivGoalPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &FootstepPlanner::goalPoseCallback, &ivFootstepPlanner);
  ivStartPoseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &FootstepPlanner::startPoseCallback, &ivFootstepPlanner);

  // service:
  ivFootstepPlanService = nh.advertiseService("plan_footsteps", &FootstepPlanner::planService, &ivFootstepPlanner);
  ivFootstepPlanFeetService = nh.advertiseService("plan_footsteps_feet", &FootstepPlanner::planFeetService, &ivFootstepPlanner);
}


FootstepPlannerNode::~FootstepPlannerNode()
{}
}
