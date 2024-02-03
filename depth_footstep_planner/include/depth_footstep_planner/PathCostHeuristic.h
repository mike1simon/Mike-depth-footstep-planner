
#ifndef DEPTH_FOOTSTEP_PLANNER_PATHCOSTHEURISTIC_H_
#define DEPTH_FOOTSTEP_PLANNER_PATHCOSTHEURISTIC_H_
#include <sbpl_edit/Depth2Dgridsearch.h>

#include <depth_footstep_planner/Heuristic.h>
#include <depthmap_humanoid_msgs/depthmap2d.h>
#include <sbpl/headers.h>
#include <depth_footstep_planner/helper.h>

namespace depth_footstep_planner
{
/**
 * @brief Determining the heuristic value by calculating a 2D path from each
 * grid cell of the map to the goal and using the path length as expected
 * distance.
 *
 * The heuristic value consists of the following factors:
 *
 *  + The expected distance retreived from the 2D path.
 *
 *  + The expected path costs.
 *
 *  + The difference between the orientation of the two states multiplied
 *    by some cost factor.
 * 
 *  + The difference between the attitude of the two states multiplied
 *    by some cost factor.
 */

#include <sbpl_edit/Depth2Dgridsearch.h>

class PathCostHeuristic : public Heuristic
{
public:
  PathCostHeuristic(double cell_size, int num_angle_bins,
                    double step_cost, double diff_angle_cost,
                    double max_step_width, int Gridsearch_downsampling, double maxStepElevation=0.1);
  PathCostHeuristic(double cell_size, int num_angle_bins,
                    double step_cost, double diff_angle_cost,double diff_depth_cost,
                    double max_step_width, int Gridsearch_downsampling, double maxStepElevation=0.1);
  PathCostHeuristic(double cell_size, int num_angle_bins,
                    double step_cost, double diff_angle_cost,double diff_depth_cost,double dist_cost,
                    double max_step_width, int Gridsearch_downsampling, double maxStepElevation=0.1);
  virtual ~PathCostHeuristic();

  /**
   * @return The estimated costs needed to reach the state 'to' from within the
   * current state.
   */
  virtual double getHValue(const PlanningState& current,
                           const PlanningState& to) const;

  /**
   * @brief Calculates for each grid cell of the map a 2D path to the
   * cell (to.x, to.y).
   * For forward planning 'to' is supposed to be the goal state, for backward
   * planning 'to' is supposed to be the start state.
   */
  bool calculateDistances(const PlanningState& from, const PlanningState& to);

  void updateMap(depthmap2d::DepthMap2DPtr map);
  void updateModelOutput(const sensor_msgs::Image::ConstPtr& model_output);
  sensor_msgs::Image getDepth2DGridSearchMsg();
  inline double getDiffDepthCost(){return ivDiffDepthCost;}

private:
  static const int cvObstacleThreshold = 200;

  unsigned char** ivpGrid;
  float** ivpDepth2D;

  double ivStepCost;
  double ivDiffAngleCost;
  double ivDiffDepthCost;
  double ivMaxStepWidth;
  double ivMaxStepElevation;
  double ivDistanceCost;
  int ivGridsearch_downsampling;

  int ivGoalX;
  int ivGoalY;

  depthmap2d::DepthMap2DPtr ivMapPtr;
  // sensor_msgs::Image::ConstPtr ivModelOutputPtr;
//  boost::shared_ptr<DEPTH2DGridSearch> ivGridSearchPtr;
  boost::shared_ptr<sbpl_edit::DEPTH2DGridSearch> ivGridSearchPtr;
  sensor_msgs::Image ivDepth2DGridSearchMsg;
//  sbpl_edit::DEPTH2DGridSearch ivGridSearchPtr;
  void resetGrid();
};
}
#endif  // DEPTH_FOOTSTEP_PLANNER_PATHCOSTHEURISTIC_H_
