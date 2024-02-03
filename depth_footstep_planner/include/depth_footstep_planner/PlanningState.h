#ifndef DEPTH_FOOTSTEP_PLANNER_PLANNINGSTATE_H
#define DEPTH_FOOTSTEP_PLANNER_PLANNINGSTATE_H

#include <depth_footstep_planner/helper.h>
#include <depth_footstep_planner/State.h>


namespace depth_footstep_planner
{
/**
 * @brief A class representing the robot's pose (i.e. position and
 * orientation) in the underlying SBPL. More precisely a planning state
 * is a discrete representation of the robot's supporting leg.
 *
 * Since SBPL is working on discretized states the planning states are also
 * discretized positions and orientations. This is done by fitting the
 * positions into a grid and the orientations into bins.
 * (NOTE: the resolution of the planning cells is likely to differ from the
 * resolution of the grid map.)
 *
 * The SBPL can access each planning state via an unique ID. Furthermore
 * each planning state can be identified by an (ununique) hash tag generated
 * from its position, location and supporting leg.
 */
class PlanningState
{
public:
  /**
   * @brief x, y and theta represent the global (continuous) position and
   * orientation of the robot's support leg.
   *
   * @param leg The supporting leg.
   * @param cell_size The size of each grid cell discretizing the
   * position.
   * @param num_angle_bins The number of bins discretizing the
   * orientation.
   * @param max_hash_size
   */
  PlanningState(double x, double y, double theta, Leg leg,
                double cell_size, int num_angle_bins, int max_hash_size);
/** @param depth is the attitude of the foot in meters */
  PlanningState(double x, double y, double theta, Leg leg,double depth,
                double cell_size, int num_angle_bins, int max_hash_size);

  /**
   * @brief x, y and theta as discrete bin values (as used internally by
   * the planner).
   */
  PlanningState(int x, int y, int theta, Leg leg, int max_hash_size);

  /** @param depth is the attitude of the foot in meters */
  PlanningState(int x, int y, int theta, Leg leg,double depth, int max_hash_size);

  /// Create a (discrete) PlanningState from a (continuous) State.
  PlanningState(const State& s, double cell_size, int num_angle_bins,
                int max_hash_size);

  /// Copy constructor.
  PlanningState(const PlanningState& s);

  ~PlanningState();

  /**
   * @brief Compare two states on equality of x, y, theta, leg. Makes
   * first use of the non-unique hash tag to rule out unequal states.
   */
  bool operator ==(const PlanningState& s2) const;

  /**
   * @brief Compare two states on inequality of x, y, theta, leg by
   * comparing the hash tags of the states.
   */
  bool operator !=(const PlanningState& s2) const;

  /**
   * @brief Used to attach such an unique ID to the planning state. (This
   * cannot be done in the constructor since often such an ID is not known
   * when the planning state is created.)
   */
  void setId(unsigned int id) { ivId = id; }
  void setDepth(double depth) { ivDepth = depth; }

  Leg getLeg() const { return ivLeg; }
  int getTheta() const { return ivTheta; }
  int getX() const { return ivX; }
  int getY() const { return ivY; }
  double getDepth() const { return ivDepth; }

  /**
   * @return The (non-unique) hash tag used to identify the planning
   * state.
   */
  unsigned int getHashTag() const { return ivHashTag; }

  /**
   * @return The (unique) ID used within the SBPL to access the
   * planning state.
   */
  int getId() const { return ivId; }

  /// @brief Computes the continuous State the PlanningState represents.
  State getState(double cell_size, int num_angle_bins) const;

private:
  /// Value of the grid cell the position's x value is fitted into.
  int ivX;
  /// Value of the grid cell the position's y value is fitted into.
  int ivY;
  /// Number of the bin the orientation is fitted into.
  int ivTheta;
  /// The supporting leg.
  Leg	ivLeg;

  /// Value of attitude (depth) in meters 
  /// (No need to discretize since it wont be used in the configuration space)
  double ivDepth;
  /// The (unique) ID of the planning state.
  int ivId;

  /**
   * The (non-unique) hash tag of the planning state. Different hash tags
   * imply that the states differ in x, y, theta, leg.
   */
  unsigned int ivHashTag;
};
}


#endif // DEPTH_FOOTSTEP_PLANNER_PLANNINGSTATE_H
