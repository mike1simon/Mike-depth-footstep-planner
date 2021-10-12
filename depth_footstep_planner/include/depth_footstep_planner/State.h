#ifndef DEPTH_FOOTSTEP_PLANNER_STATE_H
#define DEPTH_FOOTSTEP_PLANNER_STATE_H
#include <depth_footstep_planner/helper.h>

namespace depth_footstep_planner
{
/**
 * @brief A class representing the robot's pose (i.e. position and
 * orientation) in the (continuous) world view. More precisely a state
 * points to the robot's supporting leg.
 */
class State
{
public:
  State();
  State(double x, double y, double theta, Leg leg);
  State(double x, double y, double theta, Leg leg ,double depth);
  ~State();

  void setX(double x) { ivX = x; }
  void setY(double y) { ivY = y; }
  void setTheta(double theta) { ivTheta = theta; }
  void setLeg(Leg leg) { ivLeg = leg; }
  void setDepth(double depth) { ivDepth = depth; }

  double getX() const { return ivX; }
  double getY() const { return ivY; };
  double getTheta() const { return ivTheta; }
  Leg getLeg() const { return ivLeg; }
  double getDepth() const { return ivDepth; }

  /**
   * @brief Compare two states on equality of x, y, theta, leg upon
   * a certain degree of float precision.
   */
  bool operator ==(const State& s2) const;

  /**
   * @brief Inequality operator for two states (negates the equality
   * operator).
   */
  bool operator !=(const State& s2) const;

private:
  /// The robot's position in x direction.
  double ivX;
  /// The robot's position in y direction.
  double ivY;
  /// The robot's orientation.
  double ivTheta;
  /// The robot's supporting leg.
  Leg ivLeg;
  /// The robot's attitude (depth)
  double ivDepth;
};
}


#endif // DEPTH_FOOTSTEP_PLANNER_STATE_H
