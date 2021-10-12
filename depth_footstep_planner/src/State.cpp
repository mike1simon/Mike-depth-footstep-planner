#include <depth_footstep_planner/State.h>


namespace depth_footstep_planner
{
State::State()
: ivX(0.0), ivY(0.0), ivTheta(0.0), ivLeg(NOLEG), ivDepth(-1000.0)
{}


State::State(double x, double y, double theta, Leg leg)
: ivX(x), ivY(y), ivTheta(theta), ivLeg(leg), ivDepth(-1000.0)
{}

State::State(double x, double y, double theta, Leg leg, double depth)
: ivX(x), ivY(y), ivTheta(theta), ivLeg(leg), ivDepth(depth)
{}


State::~State()
{}


bool
State::operator ==(const State& s2)
const
{
  return (
      fabs(ivX - s2.getX()) < FLOAT_CMP_THR &&
      fabs(ivY - s2.getY()) < FLOAT_CMP_THR &&
      fabs(angles::shortest_angular_distance(ivTheta, s2.getTheta())) < FLOAT_CMP_THR &&
      ivLeg == s2.getLeg()
      && fabs(ivDepth - s2.getDepth()) < FLOAT_CMP_THR
          );
}


bool
State::operator !=(const State& s2)
const
{
  return not (*this == s2);
}
} // end of namespace
