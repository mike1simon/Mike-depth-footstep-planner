#ifndef DEPTH_FOOTSTEP_PLANNER_HEURISTIC_H_
#define DEPTH_FOOTSTEP_PLANNER_HEURISTIC_H_

#include <depth_footstep_planner/helper.h>
#include <depth_footstep_planner/PlanningState.h>


namespace depth_footstep_planner
{
/**
 * @brief An abstract super class providing methods necessary to be used as
 * heuristic function within the FootstepPlanner.
 */
class Heuristic
{
public:
  enum HeuristicType { EUCLIDEAN=0, EUCLIDEAN_STEPCOST=1, PATH_COST=2 };

  Heuristic(double cell_size, int num_angle_bins, HeuristicType type);
  virtual ~Heuristic();

  /**
   * @return The heuristically determined path costs to get from
   * state 'from' to state 'to' where 'to' is supposed to be the goal of
   * the planning task. (Costs are in meter.)
   */
  virtual double getHValue(const PlanningState& from,
                           const PlanningState& to) const = 0;

  HeuristicType getHeuristicType() const { return ivHeuristicType; }
  sensor_msgs::Image getDepth2DGridSearchMsg();

protected:
  double ivCellSize;
  int    ivNumAngleBins;

  const HeuristicType ivHeuristicType;
};


/**
 * @brief Determining the heuristic value by the euclidean distance between
 * two states.
 */
class EuclideanHeuristic : public Heuristic
{
public:
  EuclideanHeuristic(double cell_size, int num_angle_bins);
  virtual ~EuclideanHeuristic();

  virtual double getHValue(const PlanningState& from,
                           const PlanningState& to) const;
};


/**
 * @brief Determining the heuristic value by the euclidean distance between
 * two states, the expected step costs to get from one state to the other
 * and the difference between the orientation of the two states multiplied
 * by some cost factor. (NOTE: choosing this angular difference cost might
 * overestimate the heuristic value.)
 */
class EuclStepCostHeuristic : public Heuristic
{

public:
  EuclStepCostHeuristic(double cell_size, int num_angle_bins,
                        double step_cost, double diff_angle_cost,
                        double max_step_width);
  EuclStepCostHeuristic(double cell_size,int    num_angle_bins,
                        double step_cost,double diff_angle_cost,
                        double diff_depth_cost,double max_step_width);
  virtual ~EuclStepCostHeuristic();

  virtual double getHValue(const PlanningState& from,
                           const PlanningState& to) const;
  inline double getDiffDepthCost(){return ivDiffDepthCost;}
private:
  const double ivStepCost;
  const double ivDiffAngleCost;
  const double ivDiffDepthCost;

  /// longest step width
  const double ivMaxStepWidth;
};
}
#endif  // DEPTH_FOOTSTEP_PLANNER_HEURISTIC_H_
