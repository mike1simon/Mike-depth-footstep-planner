#include <depth_footstep_planner/PlanningStateChangeQuery.h>


namespace depth_footstep_planner
{
PlanningStateChangeQuery::PlanningStateChangeQuery(
    const std::vector<int>& neighbors)
: ivNeighbors(neighbors)
{}


PlanningStateChangeQuery::~PlanningStateChangeQuery()
{}


const std::vector<int>*
PlanningStateChangeQuery::getPredecessors() const
{
  return &ivNeighbors;
}


const std::vector<int>*
PlanningStateChangeQuery::getSuccessors() const
{
  return &ivNeighbors;
}
}
