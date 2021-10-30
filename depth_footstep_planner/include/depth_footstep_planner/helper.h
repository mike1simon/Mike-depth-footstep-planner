#ifndef DEPTH_FOOTSTEP_PLANNER_HELPER_H
#define DEPTH_FOOTSTEP_PLANNER_HELPER_H

#include <depthmap_humanoid_msgs/depthmap2d.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <math.h>

namespace depth_footstep_planner
{
static const double TWO_PI = 2 * M_PI;

static const double FLOAT_CMP_THR = 0.0001;

enum Leg { RIGHT=0, LEFT=1, NOLEG=2 };


/**
 * @return Squared euclidean distance between two integer coordinates
 * (cells).
 */
inline double euclidean_distance_sq(int x1, int y1, int x2, int y2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}


/// @return Euclidean distance between two integer coordinates (cells).
inline double euclidean_distance(int x1, int y1, int x2, int y2)
{
  return sqrt(double(euclidean_distance_sq(x1, y1, x2, y2)));
}


/// @return Euclidean distance between two coordinates.
inline double euclidean_distance(double x1, double y1, double x2, double y2)
{
  return sqrt(euclidean_distance_sq(x1, y1, x2, y2));
}


/// @return Squared euclidean distance between two coordinates.
inline double euclidean_distance_sq(double x1, double y1, double x2,
                                    double y2)
{
  // note: do *not* use pow() to square!
  return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
}


/// @return The distance of two neighbored cell.
inline double grid_cost(int x1, int y1, int x2, int y2, float cell_size)
{
  int x = abs(x1 - x2);
  int y = abs(y1 - y2);

  if (x + y > 1)
    return M_SQRT2 * cell_size;
  else
    return cell_size;
}


/// @brief Discretize a (continuous) angle into a bin.
inline int angle_state_2_cell(double angle, int angle_bin_num)
{
  double bin_size_half = M_PI / angle_bin_num;
  return int(angles::normalize_angle_positive(angle + bin_size_half) /
             TWO_PI * angle_bin_num);
}


/// @brief Calculate the respective (continuous) angle for a bin.
inline double angle_cell_2_state(int angle, int angle_bin_num)
{
  double bin_size = TWO_PI / angle_bin_num;
  return angle * bin_size;
}


/**
 * @brief Discretize a (continuous) state value into a cell. (Should be
 * used to discretize a State to a PlanningState.)
 */
inline int state_2_cell(float value, float cell_size)
{
  return value >= 0 ? int(value / cell_size) : int(value / cell_size) - 1;
}


/**
 * @brief Calculate the respective (continuous) state value for a cell.
 * (Should be used to get a State from a discretized PlanningState.)
 */
inline double cell_2_state(int value, double cell_size)
{
  return (double(value) + 0.5) * cell_size;
}


/// @brief Discretize a (continuous) value into cell size.
// TODO: check consistency for negative values
inline int disc_val(double length, double cell_size)
{
  return int(floor((length / cell_size) + 0.5));
}


/**
 * @brief Calculates the continuous value for a length discretized in cell
 * size.
 */
// TODO: check consistency for negative values
inline double cont_val(int length, double cell_size)
{
  return double(length * cell_size);
}

/// @brief Discretize a (continuous) depth value in meter into int with n decimels into cell size.
// TODO: check consistency for negative values
inline int disc_depth(double depth, int n_decimals)
{
  return int(depth*n_decimals);
}
/**
 * @brief Calculates the continuous value for a depht discretized with n decimals.
 */
// TODO: check consistency for negative values
inline double cont_depth(int depth, int n_decimals)
{
  return double(depth)/n_decimals;
}


/// @return The hash value of the key.
inline unsigned int int_hash(int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}


/**
 * @return The hash tag for a PlanningState (represented by x, y, theta and
 * leg).
 */
inline unsigned int calc_hash_tag(int x, int y, int theta, int leg,
                                  int max_hash_size)
{
  return int_hash((int_hash(x) << 3) + (int_hash(y) << 2) +
                  (int_hash(theta) << 1) + (int_hash(leg)))
      % max_hash_size;
}

inline unsigned int calc_hash_tag_with_depth(int x, int y,int depth, int theta, int leg,
                                  int max_hash_size)
{
  return int_hash((int_hash(x) << 4) + (int_hash(y) << 3) +(int_hash(depth) << 2) +
                  (int_hash(theta) << 1) + (int_hash(leg)))
      % max_hash_size;
}

/// @brief Rounding half towards zero.
inline int round(double r)
{
  return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}




/**
 * @brief Crossing number method to determine whether a point lies within a
 * polygon or not.
 * @param edges (x,y)-points defining the polygon.
 *
 * Check http://geomalgorithms.com/a03-_inclusion.html for further details.
 */
bool pointWithinPolygon(int x, int y,
                        const std::vector<std::pair<int, int> >& edges);

/**
 * @brief Checks if a footstep (represented by its center and orientation)
 * is capebale to step on the environment. The check is done by different methods
 *
 * @param x Global position of the foot in x direction.
 * @param y Global position of the foot in y direction.
 * @param theta Global orientation of the foot.
 * @param height Size of the foot in x direction.
 * @param width Size of the foot in y direction.
 * @param method (0) stable depth of the foot; (1) not implemented yet;
 * (2) not implemented yet
 * @param depth_map Contains depth information.
 *
 * @return True if the footstep collides with an obstacle.
 */
bool collision_check(int x, int y, int theta, double& new_depth,
                     int foot_height, int foot_width, int method,
                     const depthmap2d::DepthMap2D& depth_map, boost::shared_ptr<cv::Mat> map_step_validity);

/**
 * @brief first method Checks if a footstep (represented by its center and orientation)
 * is capeble of stepping in the map by counting the different hights and
 * there percentage and checking if the highest hight contain 85% of all the hights
 * and if true then it will be valid
 * @param x Global position of the foot in x direction.
 * @param y Global position of the foot in y direction.
 * @param theta Global orientation of the foot.
 * @param foot_height Size of the foot in x direction.
 * @param foot_width Size of the foot in y direction.
 * @param Map Contains depth information.
 * @param highest_hight Contains the valid highest depth and
 *  it is the depth value of the foot.
 *
 * @return True if the footstep is valid.
 */
bool check_validity_stable_depth(double x, double y, double theta,
                                 double foot_height, double foot_width
                                 ,cv::Mat Map, double &highest_hight);

double get_depth(int x, int y,cv::Mat Map);

}


#endif // DEPTH_FOOTSTEP_PLANNER_HELPER_H
