
#ifndef DEPTH_FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_
#define DEPTH_FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_


#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <depth_footstep_planner/FootstepPlanner.h>


namespace depth_footstep_planner
{
/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality.
 */
class FootstepPlannerNode
{
public:
  FootstepPlannerNode();
  virtual ~FootstepPlannerNode();

protected:
  FootstepPlanner ivFootstepPlanner;

  ros::Subscriber ivGoalPoseSub;
  ros::Subscriber ivGridMapSub;
  ros::Subscriber ivGridModelOutputSub;
  ros::Subscriber ivStartPoseSub;
  ros::Subscriber ivRobotPoseSub;

  ros::ServiceServer ivFootstepPlanService;
  ros::ServiceServer ivFootstepPlanFeetService;
};
}
#endif  // DEPTH_FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_
