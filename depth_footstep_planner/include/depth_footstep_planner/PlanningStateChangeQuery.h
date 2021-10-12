#ifndef DEPTH_FOOTSTEP_PLANNER_PLANNINGSTATECHANGEQUERY_H_
#define DEPTH_FOOTSTEP_PLANNER_PLANNINGSTATECHANGEQUERY_H_


#include <vector>

#include <sbpl/headers.h>


namespace depth_footstep_planner
{
class PlanningStateChangeQuery : public StateChangeQuery
{
public:
  PlanningStateChangeQuery(const std::vector<int>& neighbors);
  virtual ~PlanningStateChangeQuery();

  const std::vector<int>* getPredecessors() const;
  const std::vector<int>* getSuccessors() const;

private:
  const std::vector<int>& ivNeighbors;
};
}

#endif  // DEPTH_FOOTSTEP_PLANNER_PLANNINGSTATECHANGEQUERY_H_
