#ifndef DEPTH_FOOTSTEP_PLANNER_FOOTSTEPPLANNERENVIRONMENT_H_
#define DEPTH_FOOTSTEP_PLANNER_FOOTSTEPPLANNERENVIRONMENT_H_

#include <depth_footstep_planner/helper.h>
#include <depth_footstep_planner/PathCostHeuristic.h>
#include <depth_footstep_planner/Heuristic.h>
#include <depth_footstep_planner/Footstep.h>
#include <depth_footstep_planner/PlanningState.h>
#include <depth_footstep_planner/State.h>
#include <humanoid_nav_msgs/ClipFootstep.h>
#include <sbpl/headers.h>

#include <math.h>
#include <vector>
#include <tr1/unordered_set>
#include <tr1/hashtable.h>


namespace depth_footstep_planner
{
struct environment_params
{
  std::vector<Footstep> footstep_set;
  boost::shared_ptr<Heuristic> heuristic;

  /// Defines the area of performable (discrete) steps.
  std::vector<std::pair<int, int> > step_range;

  double footsize_x, footsize_y, footsize_z;
  double foot_origin_shift_x, foot_origin_shift_y;
  double max_footstep_x, max_footstep_y, max_footstep_theta;
  double max_inverse_footstep_x, max_inverse_footstep_y,
         max_inverse_footstep_theta;
  double step_cost;
  int gridsearch_downsampling;
//  int    collision_check_accuracy;
  int    collision_check_method;
  int    hash_table_size;
  double cell_size;
  int    num_angle_bins;
  bool   forward_search;
  double max_step_width;
  int    num_random_nodes;
  double random_node_distance;
  double heuristic_scale;
  double MaxAttitudeForStep;
};


/**
 * @brief A class defining a footstep planner environment for humanoid
 * robots used by the SBPL to perform planning tasks.
 *
 * The environment keeps track of all the planning states expanded during
 * the search. Each planning state can be accessed via its ID. Furthermore
 */
class FootstepPlannerEnvironment : public DiscreteSpaceInformation
{
public:
  // specialization of hash<int,int>, similar to standard boost::hash on pairs?
  struct IntPairHash{
  public:
    size_t operator()(std::pair<int, int> x) const throw() {
      size_t seed = std::tr1::hash<int>()(x.first);
      return std::tr1::hash<int>()(x.second) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }
  };

  typedef std::vector<int> exp_states_t;
  typedef exp_states_t::const_iterator exp_states_iter_t;
  typedef std::tr1::unordered_set<std::pair<int,int>, IntPairHash > exp_states_2d_t;
  typedef exp_states_2d_t::const_iterator exp_states_2d_iter_t;

  /**
   * @param footstep_set The set of footsteps used for the path planning.
   * @param heuristic The heuristic used by the planner.
   * @param footsize_x Size of the foot in x direction.
   * @param footsize_y Size of the foot in y direction.
   * @param origin_foot_shift_x Shift in x direction from the foot's
   * center.
   * @param origin_foot_shift_y Shift in y direction from the foot's
   * center.
   * @param max_footstep_x The maximal translation in x direction
   * performable by the robot.
   * @param max_footstep_y The maximal translation in y direction
   * performable by the robot.
   * @param max_footstep_theta The maximal rotation performable by the
   * robot.
   * @param max_inverse_footstep_x The minimal translation in x direction
   * performable by the robot.
   * @param max_inverse_footstep_y The minimal translation in y direction
   * performable by the robot.
   * @param max_inverse_footstep_theta The minimal rotation performable by
   * the robot.
   * @param step_cost The costs for each step.
   * @param gridsearch_downsampling downsampling for GridSearch map.
   * @param collision_check_accuracy Whether to check just the foot's
   * circumcircle (0), the incircle (1) or recursively the circumcircle
   * and the incircle for the whole foot (2) for collision.
   * @param hash_table_size Size of the hash table storing the planning
   * states expanded during the search.
   * @param cell_size The size of each grid cell used to discretize the
   * robot positions.
   * @param num_angle_bins The number of bins used to discretize the
   * robot orientations.
   * @param forward_search Whether to use forward search (1) or backward
   * search (0).
   */
  FootstepPlannerEnvironment(const environment_params& params);

  virtual ~FootstepPlannerEnvironment();

  /**
   * @brief Update the robot's feet poses in the goal state.
   * @return The new IDs (left, right) of the planning state representing the
   * feet.
   */
  std::pair<int, int> updateGoal(const State& foot_left,
                                 const State& foot_right);

  /**
   * @brief Update the robot's feet poses in the start state.
   * @return The new IDs (left, right) of the planning states representing the
   * feet.
   */
  std::pair<int, int> updateStart(const State& foot_left,
                                  const State& right_right);

  //  void updateMap(gridmap_2d::GridMap2DPtr map);
  void updateMap(depthmap2d::DepthMap2DPtr map);
  void updateModelOutput(const sensor_msgs::Image::ConstPtr& model_output);

  /**
   * @return True iff the foot in State s is colliding with an
   * obstacle.
   */
  bool occupied(State& s);

  /**
   * @brief Try to receive a state with a certain ID.
   *
   * @return True iff there is a state with such an ID.
   */
  bool getState(unsigned int id, State* s);

  /**
   * @brief Resets the current planning task (i.e. the start and goal
   * poses).
   */
  void reset();

  /// @return The number of expanded states during the search.
  int getNumExpandedStates() { return ivNumExpandedStates; }
  sensor_msgs::Image getDepth2DGridSearchMsg();
  exp_states_2d_iter_t getExpandedStatesStart()
  {
    return ivExpandedStates.begin();
  }

  exp_states_2d_iter_t getExpandedStatesEnd()
  {
    return ivExpandedStates.end();
  }

  exp_states_iter_t getRandomStatesStart()
  {
    return ivRandomStates.begin();
  }

  exp_states_iter_t getRandomStatesEnd()
  {
    return ivRandomStates.end();
  }

  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(int FromStateID, int ToStateID);

  /**
   * @return The heuristic value to reach the goal from within the
   * planning state stateID (used for forward planning).
   */
  int GetGoalHeuristic(int stateID);

  /**
   * @return The heuristic value to reach the start from within
   * the planning state stateID. (Used for backward planning.)
   */
  int GetStartHeuristic(int stateID);

  /**
   * @brief Calculates the successor states and the corresponding costs
   * when performing the footstep set on the planning state SourceStateID.
   * (Used for forward planning.)
   */
  void GetSuccs(int SourceStateID, std::vector<int> *SuccIDV,
                std::vector<int> *CostV);

  /**
   * @brief Calculates the predecessor states and the corresponding costs
   * when reversing the footstep set on the planning state TargetStateID.
   * (Used for backward planning.)
   */
  void GetPreds(int TargetStateID, std::vector<int> *PredIDV,
                std::vector<int> *CostV);

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  virtual void GetRandomSuccsatDistance(int SourceStateID,
                                        std::vector<int>* SuccIDV,
                                        std::vector<int>* CLowV);

  /**
   * @brief Used for RStar: generate succs/preds at some
   * domain-dependent distance. The number of generated succs/preds is up
   * to the environment.
   */
  virtual void GetRandomPredsatDistance(int TargetStateID,
                                        std::vector<int>* PredIDV,
                                        std::vector<int>* CLowV);

  /// Testing, for R*
  void GetSuccsTo(int SourceStateID, int goalStateID,
                  std::vector<int> *SuccIDV, std::vector<int> *CostV);

  /// @return True if two states meet the same condition. Used for R*.
  bool AreEquivalent(int StateID1, int StateID2);

  bool InitializeEnv(const char *sEnvFile);

  bool InitializeMDPCfg(MDPConfig *MDPCfg);

  void PrintEnv_Config(FILE *fOut);

  void PrintState(int stateID, bool bVerbose, FILE *fOut);

  void SetAllActionsandAllOutcomes(CMDPSTATE *state);

  void SetAllPreds(CMDPSTATE *state);

  int SizeofCreatedEnv();

  /**
   * @return True iff 'to' can be reached by an arbitrary footstep that
   * can be performed by the robot from within 'from'. (This method is
   * used to check whether the goal/start can be reached from within the
   * current state.)
   */
  bool reachable(const PlanningState& from, const PlanningState& to);
  bool reachableFromDepth(const PlanningState& from, const PlanningState& to);

  void getPredsOfGridCells(const std::vector<State>& changed_states,
                           std::vector<int>* pred_ids);

  void getSuccsOfGridCells(const std::vector<State>& changed_states,
                           std::vector<int>* succ_ids);

  /**
   * @brief Update the heuristic values (e.g. after the map has changed).
   * The environment takes care that the update is only done when it is
   * necessary.
   */
  void updateHeuristicValues();

  /// Used to scale continuous values in meter to discrete values in mm.
  static const int cvMmScale = 1000;

  inline int getivdebug(){return ivdebug;}

protected:
  /**
   * @return The costs (in mm, truncated as int) to reach the
   * planning state ToStateID from within planning state FromStateID.
   */
  int GetFromToHeuristic(const PlanningState& from, const PlanningState& to);

  /// @return The step cost for reaching 'b' from within 'a'.
  int  stepCost(const PlanningState& a, const PlanningState& b);

  /**
   * @return True iff the foot in 's' is colliding with an obstacle.
   */
  bool occupied(PlanningState& s);

  void GetRandomNeighs(const PlanningState* currentState,
                       std::vector<int>* NeighIDV,
                       std::vector<int>* CLowV,
                       int nNumofNeighs, int nDist_c, bool bSuccs);

  void setStateArea(const PlanningState& left, const PlanningState& right);

  /// Wrapper for FootstepPlannerEnvironment::createNewHashEntry(PlanningState).
  const PlanningState* createNewHashEntry(const State& s);

  /**
   * @brief Creates a new planning state for 's' and inserts it into the
   * maps (PlanningState::ivStateId2State,
   * PlanningState::ivpStateHash2State)
   *
   * @return A pointer to the newly created PlanningState.
   */
  const PlanningState* createNewHashEntry(const PlanningState& s);

  /// Wrapper for FootstepPlannerEnvironment::getHashEntry(PlanningState).
  const PlanningState* getHashEntry(const State& s);

  /**
   * @return The pointer to the planning state 's' stored in
   * FootstepPlannerEnvironment::ivpStateHash2State.
   */
  const PlanningState* getHashEntry(const PlanningState& s);

  const PlanningState* createHashEntryIfNotExists(const PlanningState& s);

  /**
   * @return True iff 'goal' can be reached by an arbitrary footstep.
   * (Used for forward planning.)
   */
  bool closeToGoal(const PlanningState& from);

  /**
   * @return True iff 'start' can be reached by an arbitrary footstep.
   * (Used for backward planning.)
   */
  bool closeToStart(const PlanningState& from);


  /// < operator for planning states.
  struct less
  {
    bool operator ()(const PlanningState* a,
                     const PlanningState* b) const;
  };

  /**
   * @brief ID of the planning goal, i.e. dependent on the planning direction
   * (forward/backward) this ID is used to map to the goal/start poses.
   */
  int ivIdPlanningGoal;

  /// ID of the start pose of the left foot.
  int ivIdStartFootLeft;
  /// ID of the start pose of the right foot.
  int ivIdStartFootRight;
  /// ID of the goal pose of the left foot.
  int ivIdGoalFootLeft;
  /// ID of the goal pose of the right foot.
  int ivIdGoalFootRight;

  std::vector<int> ivStateArea;

  /**
   * @brief Maps from an ID to the corresponding PlanningState. (Used in
   * the SBPL to access a certain PlanningState.)
   */
  std::vector<const PlanningState*> ivStateId2State;
  //std::vector<PlanningState*> ivStateId2State;

  /**
   * @brief Maps from a hash tag to a list of corresponding planning
   * states. (Used in FootstepPlannerEnvironment to identify a certain
   * PlanningState.)
   */
  std::vector<const PlanningState*>* ivpStateHash2State;

  /// The set of footsteps used for the path planning.
  const std::vector<Footstep>& ivFootstepSet;

  /// The heuristic function used by the planner.
  const boost::shared_ptr<Heuristic> ivHeuristicConstPtr;

  /// Size of the foot in x direction.
  const double ivFootsizeX;
  /// Size of the foot in y direction.
  const double ivFootsizeY;

  /// Shift in x direction from the foot's center.
  const double ivOriginFootShiftX;
  /// Shift in y direction from the foot's center.
  const double ivOriginFootShiftY;

  /// The maximal translation in x direction (discretized in cell size).
  const int ivMaxFootstepX;
  /// The maximal translation in y direction (discretized in cell size).
  const int ivMaxFootstepY;
  /// The maximal rotation (discretized into bins).
  int ivMaxFootstepTheta;

  /// The minimal translation in x direction (discretized in cell size).
  const int ivMaxInvFootstepX;
  /// The minimal translation in y direction (discretized in cell size).
  const int ivMaxInvFootstepY;
  /// The minimal rotation (discretized into bins).
  int ivMaxInvFootstepTheta;

  double ivMaxAttitudeForStep;

  /**
   * @brief The costs for each step (discretized with the help of
   * cvMmScale).
   */
  const int ivStepCost;
  const int ivGridsearch_downsampling;
  /**
   * @brief Whether to check just the foot's inner circle (0), the hole
   * outer circle (1) or exactly the foot's bounding box (2) for
   * collision.
   */
//  const int ivCollisionCheckAccuracy;
  const int ivCollisionCheckMethod;

  /**
   * @brief Size of the hash table storing the planning states expanded
   * during the search. (Also referred to by max_hash_size.)
   */
  const int ivHashTableSize;

  /// The size of each grid cell used to discretize the robot positions.
  const double ivCellSize;
  /// The number of bins used to discretize the robot orientations.
  const int ivNumAngleBins;

  /// Whether to use forward search (1) or backward search (0).
  const bool ivForwardSearch;

  double ivMaxStepWidth;

  /// number of random neighbors for R*
  const int ivNumRandomNodes;
  /// distance of random neighbors for R* (discretized in cells)
  const int ivRandomNodeDist;

  /**
   * Scaling factor of heuristic, in case it underestimates by a constant
   * factor.
   */
  double ivHeuristicScale;

  /// Indicates if heuristic has to be updated.
  bool ivHeuristicExpired;

  /// Pointer to the map.
  //  boost::shared_ptr<gridmap_2d::GridMap2D> ivMapPtr;
  boost::shared_ptr<depthmap2d::DepthMap2D> ivMapPtr;
  // boost::shared_ptr<sensor_msgs::Image> ivModelOutputPtr;
  boost::shared_ptr<cv::Mat> ivModelOutputPtr;
  
  exp_states_2d_t ivExpandedStates;
  exp_states_t ivRandomStates;  ///< random intermediate states for R*
  size_t ivNumExpandedStates;

  bool* ivpStepRange;
  int ivdebug=0;
};
}

#endif  // DEPTH_FOOTSTEP_PLANNER_FOOTSTEPPLANNERENVIRONMENT_H_
