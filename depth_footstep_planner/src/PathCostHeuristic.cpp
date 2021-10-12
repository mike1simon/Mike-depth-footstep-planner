
#include <depth_footstep_planner/PathCostHeuristic.h>


namespace depth_footstep_planner
{
PathCostHeuristic::PathCostHeuristic(double cell_size,
                                     int    num_angle_bins,
                                     double step_cost,
                                     double diff_angle_cost,
                                     double max_step_width,
                                     double inflation_radius)
: Heuristic(cell_size, num_angle_bins, PATH_COST),
//  ivpGrid(NULL),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivDiffDepthCost(0.0),
  ivMaxStepWidth(max_step_width),
  ivDistanceCost(1.0),
  ivInflationRadius(inflation_radius),
  ivGoalX(-1),
  ivGoalY(-1)
{}
PathCostHeuristic::PathCostHeuristic(double cell_size, int num_angle_bins,
                  double step_cost, double diff_angle_cost,double diff_depth_cost,
                  double max_step_width, double inflation_radius)
  : Heuristic(cell_size, num_angle_bins, PATH_COST),
//    ivpGrid(NULL),
    ivStepCost(step_cost),
    ivDiffAngleCost(diff_angle_cost),
    ivDiffDepthCost(diff_depth_cost),
    ivMaxStepWidth(max_step_width),
    ivDistanceCost(1.0),
    ivInflationRadius(inflation_radius),
    ivGoalX(-1),
    ivGoalY(-1)
  {}
PathCostHeuristic::PathCostHeuristic(double cell_size, int num_angle_bins,
                  double step_cost, double diff_angle_cost,double diff_depth_cost,double dist_cost,
                  double max_step_width, double inflation_radius)
  : Heuristic(cell_size, num_angle_bins, PATH_COST),
//    ivpGrid(NULL),
    ivStepCost(step_cost),
    ivDiffAngleCost(diff_angle_cost),
    ivDiffDepthCost(diff_depth_cost),
    ivDistanceCost(dist_cost),
    ivMaxStepWidth(max_step_width),
    ivInflationRadius(inflation_radius),
    ivGoalX(-1),
    ivGoalY(-1)
  {}


PathCostHeuristic::~PathCostHeuristic()
{
//  if (ivpGrid)
//    resetGrid();
}


double
PathCostHeuristic::getHValue(const PlanningState& current,
                             const PlanningState& to)
const
{
  assert(ivGoalX >= 0 && ivGoalY >= 0);

  if (current == to)
    return 0.0;

  unsigned int from_x;
  unsigned int from_y;
  // could be removed after more testing (then use ...noBounds... again)
//  ivMapPtr->worldToMapNoBounds(cell_2_state(current.getX(), ivCellSize),
//                               cell_2_state(current.getY(), ivCellSize),
//                               from_x, from_y);
  ivMapPtr->worldToMap(cell_2_state(current.getX(), ivCellSize),
                               cell_2_state(current.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  // could be removed after more testing (then use ...noBounds... again)
//  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
//                               cell_2_state(to.getY(), ivCellSize),
//                               to_x, to_y);
  ivMapPtr->worldToMap(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  // cast to unsigned int is safe since ivGoalX/ivGoalY are checked to be >= 0
  if ((unsigned int)ivGoalX != to_x || (unsigned int)ivGoalY != to_y)
  {
    ROS_ERROR("PathCostHeuristic::getHValue to a different value than "
              "precomputed, heuristic values will be wrong. You need to call "
              "calculateDistances() before!");
  }
  assert((unsigned int)ivGoalX == to_x && (unsigned int)ivGoalY == to_y);
//  no grid search needed
//  double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(
//      from_x, from_y)) / 1000.0;
//  double dist = euclidean_distance(int(from_x),int(from_y),int(to_x),int(to_y));
//  double dist = double(euclidean_distance(int(from_x),int(from_y),int(to_x),int(to_y)))/1000.0;
  double dist = euclidean_distance(int(from_x),int(from_y),int(to_x),int(to_y)) * ivCellSize;
  double expected_steps = dist / ivMaxStepWidth;
  double diff_angle = 0.0;
  double diff_depth = 0.0;
  if (ivDiffAngleCost > 0.0)
  {
    // get the number of bins between from.theta and to.theta
    int diff_angle_disc = (
        ((to.getTheta() - current.getTheta()) % ivNumAngleBins) +
        ivNumAngleBins) % ivNumAngleBins;
    // get the rotation independent from the rotation direction
    diff_angle = std::abs(angles::normalize_angle(
        angle_cell_2_state(diff_angle_disc, ivNumAngleBins)));
  }
  if(ivDiffDepthCost > 0.0){
    diff_depth = fabs(current.getDepth() - to.getDepth());
  }
//  ROS_ERROR("Step cost between a[x: %d , y: %d] and b[x: %d , y: %d] is dist: %lf ivStepCost: %lf es: %lf all: %lf with depth: %lf"
//            ,from_x,from_y,to_x,to_y,dist,ivStepCost,
//            expected_steps,
//            (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost),
//            (2*dist + expected_steps * ivStepCost +
//                  diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost));

  return (ivDistanceCost*dist + expected_steps * ivStepCost +
      diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost);

//  return (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost);
}


bool
PathCostHeuristic::calculateDistances(const PlanningState& from,
                                      const PlanningState& to)
{
  assert(ivMapPtr);

  unsigned int from_x;
  unsigned int from_y;
//  ivMapPtr->worldToMapNoBounds(cell_2_state(from.getX(), ivCellSize),
//                               cell_2_state(from.getY(), ivCellSize),
//                               from_x, from_y);
  ivMapPtr->worldToMap(cell_2_state(from.getX(), ivCellSize),
                               cell_2_state(from.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
//  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
//                               cell_2_state(to.getY(), ivCellSize),
//                               to_x, to_y);
  ivMapPtr->worldToMap(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  if ((int)to_x != ivGoalX || (int)to_y != ivGoalY)
  {
    ivGoalX = to_x;
    ivGoalY = to_y;
//    ivGridSearchPtr->search(ivpGrid, cvObstacleThreshold,
//                            ivGoalX, ivGoalY, from_x, from_y,
//                            SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }

  return true;
}


void
PathCostHeuristic::updateMap(depthmap2d::DepthMap2DPtr map)
{
//  if (ivpGrid) // reset map before change it's sizes (in other case we will get SEGMENT ERROR)
//    resetGrid();

  ivMapPtr.reset();
  ivMapPtr = map;

  ivGoalX = ivGoalY = -1;

  unsigned width = ivMapPtr->getInfo().width;
  unsigned height = ivMapPtr->getInfo().height;

//  if (ivGridSearchPtr)
//    ivGridSearchPtr->destroy();
//  ivGridSearchPtr.reset(new SBPL2DGridSearch(width, height,
//                                             ivMapPtr->getResolution()));

//  ivpGrid = new unsigned char* [width];

//  for (unsigned x = 0; x < width; ++x)
//    ivpGrid[x] = new unsigned char [height];
//  for (unsigned y = 0; y < height; ++y)
//  {
//    for (unsigned x = 0; x < width; ++x)
//    {
//      float dist = 0;//ivMapPtr->distanceMapAtCell(x,y);
//      if (dist < 0.0f)
//        ROS_ERROR("Distance map at %d %d out of bounds", x, y);
//      else if (dist <= ivInflationRadius)
//        ivpGrid[x][y] = 255;
//      else
//        ivpGrid[x][y] = 0;
//    }
//  }
}


void
PathCostHeuristic::resetGrid()
{
  // CvSize size = ivMapPtr->size(); // here we get (height; width) instead of (width; height)
//  int width = ivMapPtr->getInfo().width;
//  for (int x = 0; x < width; ++x)
//  {
//    if (ivpGrid[x])
//    {
//      delete[] ivpGrid[x];
//      ivpGrid[x] = NULL;
//    }
//  }
//  delete[] ivpGrid;
//  ivpGrid = NULL;
}
} // end of namespace
