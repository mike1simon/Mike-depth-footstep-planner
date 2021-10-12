#include <depth_footstep_planner/Heuristic.h>

namespace depth_footstep_planner
{
Heuristic::Heuristic(double cell_size, int num_angle_bins,
                     HeuristicType type)
: ivCellSize(cell_size),
  ivNumAngleBins(num_angle_bins),
  ivHeuristicType(type)
{}


Heuristic::~Heuristic()
{}


EuclideanHeuristic::EuclideanHeuristic(double cell_size, int num_angle_bins)
: Heuristic(cell_size, num_angle_bins, EUCLIDEAN)
{}


EuclideanHeuristic::~EuclideanHeuristic()
{}


double
EuclideanHeuristic::getHValue(const PlanningState& from,
                              const PlanningState& to)
const
{
  if (from == to)
    return 0.0;

  // distance in cell size
  double dist = euclidean_distance(from.getX(), from.getY(),
                                   to.getX(), to.getY());
  // return distance in meter
  return cont_val(dist, ivCellSize);
}


EuclStepCostHeuristic::EuclStepCostHeuristic(double cell_size,
                                             int    num_angle_bins,
                                             double step_cost,
                                             double diff_angle_cost,
                                             double max_step_width)
: Heuristic(cell_size, num_angle_bins, EUCLIDEAN_STEPCOST),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivDiffDepthCost(0.0),
  ivMaxStepWidth(max_step_width)
{}

EuclStepCostHeuristic::EuclStepCostHeuristic(double cell_size,
                                             int    num_angle_bins,
                                             double step_cost,
                                             double diff_angle_cost,
                                             double diff_depth_cost,
                                             double max_step_width)
: Heuristic(cell_size, num_angle_bins, EUCLIDEAN_STEPCOST),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivDiffDepthCost(diff_depth_cost),
  ivMaxStepWidth(max_step_width)
{}


EuclStepCostHeuristic::~EuclStepCostHeuristic()
{}


double
EuclStepCostHeuristic::getHValue(const PlanningState& from,
                                 const PlanningState& to)
const
{
  if (from == to)
    return 0.0;

  // distance in meter
  double dist = cont_val(euclidean_distance(
      from.getX(), from.getY(), to.getX(), to.getY()), ivCellSize);
  double expected_steps = dist / ivMaxStepWidth;
  double diff_angle = 0.0;
  double diff_depth = 0.0;
  if (ivDiffAngleCost > 0.0)
  {
    // get the number of bins between from.theta and to.theta
    int diff_angle_disc = (
        ((to.getTheta() - from.getTheta()) % ivNumAngleBins) +
        ivNumAngleBins) % ivNumAngleBins;
    // get the rotation independent from the rotation direction
    diff_angle = std::abs(angles::normalize_angle(
        angle_cell_2_state(diff_angle_disc, ivNumAngleBins)));
  }

  if(ivDiffDepthCost > 0.0){
    diff_depth = fabs(from.getDepth() - to.getDepth());
  }
  return (dist + expected_steps * ivStepCost +
      diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost);
//  return (dist + expected_steps * ivStepCost +
//      diff_angle * ivDiffAngleCost);
}
}
