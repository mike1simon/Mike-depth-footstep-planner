/**
 *  TODO: 
 *    1. reConfigure the Maps and input and output
 *  ! 2. Modefy the GridSearch to get hieght and reachability
 *  ? 3. Modefy the GridSearch to merge hiegth data with reachability data of foot
 *  ? 4. Link the Train UNET model with the planner
 *  * 5. Try to modify the planner to not link two steps between or trough hazarduos area
 * **/

#include <depth_footstep_planner/PathCostHeuristic.h>
// TESTING 2DGRIDSEARCH
#include <sbpl_edit/Depth2Dgridsearch.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace depth_footstep_planner
{
PathCostHeuristic::PathCostHeuristic(double cell_size,
                                     int    num_angle_bins,
                                     double step_cost,
                                     double diff_angle_cost,
                                     double max_step_width,
                                     double inflation_radius)
: Heuristic(cell_size, num_angle_bins, PATH_COST),
  ivpGrid(NULL),
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
    ivpGrid(NULL),
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
    ivpGrid(NULL),
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
  if (ivpGrid)
    resetGrid();
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
  ivMapPtr->worldToMap(cell_2_state(current.getX(), ivCellSize),
                               cell_2_state(current.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  ivMapPtr->worldToMap(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

/**
 *  TODO: 
 *    1. Make Sure that the dist is calculated optimally
 *  ?  3. Change the hard coded 0.8 to a parameter.
 * **/
  // ! Debugging calculating the distance 
  double dist = 0.8*double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(
      from_x/2, from_y/2)) / 1000.0;
  //  * Other ways to calculated the distance instead of using GridSearch 
  //  * but (it Will get Stuck in local Minima)
  //  double dist = euclidean_distance(int(from_x),int(from_y),int(to_x),int(to_y));
  //  double dist = double(euclidean_distance(int(from_x),int(from_y),int(to_x),int(to_y)))/1000.0;
  //  double dist = euclidean_distance(int(from_x),int(from_y),int(to_x),int(to_y)) * ivCellSize;
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
  return (ivDistanceCost*dist + expected_steps * ivStepCost +
      diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost);
  // ! OLD 
  //  return (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost);
}


bool
PathCostHeuristic::calculateDistances(const PlanningState& from,
                                      const PlanningState& to)
{
  assert(ivMapPtr);
  double from_x_double = cell_2_state(from.getX(), ivCellSize);
  double from_y_double = cell_2_state(from.getY(), ivCellSize);
  double to_x_double = cell_2_state(to.getX(), ivCellSize);
  double to_y_double = cell_2_state(to.getY(), ivCellSize);


  unsigned int from_x;
  unsigned int from_y;
  bool success_from = ivMapPtr->worldToMap(from_x_double,from_y_double,
                               from_x, from_y);
  unsigned int to_x;
  unsigned int to_y;
  bool success_to = ivMapPtr->worldToMap(to_x_double,to_y_double,
                               to_x, to_y);
  ros::Time start = ros::Time::now();
  if ((int)to_x != ivGoalX || (int)to_y != ivGoalY)
  {
    ivGoalX = to_x;
    ivGoalY = to_y;
    unsigned int max_hight;
    ivGridSearchPtr->search(ivpGrid,ivpDepth2D, cvObstacleThreshold,
                            ivGoalX/2, ivGoalY/2, from_x/2, from_y/2,
                            max_hight, sbpl_edit::SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }
  double d = (ros::Time::now() - start).toSec();
  ROS_INFO("Depth2DGridSearch Done in: %3.4lf seconds",d);
  // ! GRID2DSEARCH Publish
  int width = static_cast<int>(ivMapPtr->getInfo().width/2);
  int height = static_cast<int>(ivMapPtr->getInfo().height/2);
  double maximum = -1000;
  double minimum = 100000000;
  double max_cost = -1000000.0;
  cv::Mat distances = cv::Mat::zeros(width,height,CV_32F);
  for (int i=0;i<width;i++) {
    for(int j=0;j<height;j++){
      double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(i, j)) / 1000.0;

      if(dist < 1000000.0)
        distances.at<float>(i,j) = dist;
      else
        distances.at<float>(i,j) = -1.0;
      if(dist > maximum)
        maximum = dist;
      if(dist < minimum)
        minimum = dist;
      if(dist < 1000000.0 && dist > max_cost)
        max_cost = dist;
    }
  }
  cv::Mat DEPTH2DGRIDSEARCH_image = cv::Mat::zeros(width,height,CV_8UC1);
  for (int i=0;i<width;i++) {
    for(int j=0;j<height;j++){
      if(distances.at<float>(i,j) < 0){
        distances.at<float>(i,j) = 1.0;
      }
      else
        distances.at<float>(i,j) = distances.at<float>(i,j)/(max_cost);
      DEPTH2DGRIDSEARCH_image.at<u_int8_t>(i,j) = static_cast<u_int8_t>(255 * distances.at<float>(i,j));
    }
  }
  cv_bridge::CvImage CV_Bridge;
  CV_Bridge.image = DEPTH2DGRIDSEARCH_image;
  CV_Bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  CV_Bridge.header.frame_id = "world";
  CV_Bridge.header.stamp = ros::Time::now();
  ivDepth2DGridSearchMsg = *CV_Bridge.toImageMsg();

  return true;
}

sensor_msgs::Image PathCostHeuristic::getDepth2DGridSearchMsg(){
  return ivDepth2DGridSearchMsg;
}


void
PathCostHeuristic::updateMap(depthmap2d::DepthMap2DPtr map)
{

  ivMapPtr.reset();
  ivMapPtr = map;

  if (ivpGrid) // reset map before change it's sizes (in other case we will get SEGMENT ERROR)
    resetGrid();

  ivGoalX = ivGoalY = -1;

  // TESTING GRIDSEARCH
  unsigned width = ivMapPtr->getInfo().width/2;
  unsigned height = ivMapPtr->getInfo().height/2;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();

  ivGridSearchPtr.reset(new sbpl_edit::DEPTH2DGridSearch(width, height,
                                             ivMapPtr->getResolution(), 4.0/65536.0) );
  ivpDepth2D = new float* [width];

  for (unsigned x = 0; x < width; ++x){
    ivpDepth2D[x] = new float [height];
  }
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      ivpDepth2D[x][y] = ivMapPtr->depthMapAtCell(x*2,y*2);
    }
  }


}

void PathCostHeuristic::updateModelOutput(const sensor_msgs::Image::ConstPtr& model_output)
{
  // ivModelOutputPtr.reset();
  // ivModelOutputPtr = model_output;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(model_output, model_output->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (ivpGrid) // reset map before change it's sizes (in other case we will get SEGMENT ERROR)
    resetGrid();

  ivGoalX = ivGoalY = -1;

  // TESTING GRIDSEARCH
  unsigned width = model_output->width;
  unsigned height = model_output->height;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();

  ivGridSearchPtr.reset(new sbpl_edit::DEPTH2DGridSearch(width, height,
                                             ivMapPtr->getResolution(), 4.0/65536.0 ) );

  // ? Beginning To Fill the Grid Search Array from the Neural Network Output
  cv::Mat output = cv_ptr->image;
  ROS_INFO("Depth Footstep Planning Node Got Feasible Footsteps For the Map (Model Output). \n");

  // ? Initializing Gird array from Model Output for GridSearch
  ivpGrid = new unsigned char* [width];
  for (unsigned x = 0; x < width; ++x){
    ivpGrid[x] = new unsigned char [height];
  }
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      // int output_state = static_cast<int>(output.at<unsigned short>(x,y));
      //  int output_state = static_cast<int>(output.at<unsigned short>(x,y));
      // the new flip of x and y for the search to axis the dimentions as [x, y]
      int output_state = static_cast<int>(output.at<unsigned short>(y, x));
      if (output_state < 0)
        ROS_ERROR("output_state of Model Output of map at %d %d is: %d out of bounds", x, y,output_state);
      else if (output_state <= 100)
        ivpGrid[x][y] = 255;
      else
        ivpGrid[x][y] = 0;
    }
  }


}


void
PathCostHeuristic::resetGrid()
{
  int width = ivMapPtr->getInfo().width/2;
  for (int x = 0; x < width; ++x)
  {
    if (ivpGrid[x])
    {
      delete[] ivpGrid[x];
      ivpGrid[x] = NULL;
    }
  }
  delete[] ivpGrid;
  ivpGrid = NULL;

}
} // end of namespace
