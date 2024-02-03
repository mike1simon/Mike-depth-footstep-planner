/**
 *  TODO: 
 *  * Try to modify the planner to not link two steps between or trough hazarduos area
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
                                     int Gridsearch_downsampling,
                                     double maxStepElevation)
: Heuristic(cell_size, num_angle_bins, PATH_COST),
  ivpGrid(NULL),
  ivStepCost(step_cost),
  ivDiffAngleCost(diff_angle_cost),
  ivDiffDepthCost(0.0),
  ivMaxStepWidth(max_step_width),
  ivMaxStepElevation(maxStepElevation),
  ivDistanceCost(1.0),
  ivGridsearch_downsampling(Gridsearch_downsampling),
  ivGoalX(-1),
  ivGoalY(-1)
{}
PathCostHeuristic::PathCostHeuristic(double cell_size, int num_angle_bins,
                  double step_cost, double diff_angle_cost,double diff_depth_cost,
                  double max_step_width, int Gridsearch_downsampling, double maxStepElevation)
  : Heuristic(cell_size, num_angle_bins, PATH_COST),
    ivpGrid(NULL),
    ivStepCost(step_cost),
    ivDiffAngleCost(diff_angle_cost),
    ivDiffDepthCost(diff_depth_cost),
    ivMaxStepWidth(max_step_width),
    ivMaxStepElevation(maxStepElevation),
    ivDistanceCost(1.0),
    ivGridsearch_downsampling(Gridsearch_downsampling),
    ivGoalX(-1),
    ivGoalY(-1)
  {}
PathCostHeuristic::PathCostHeuristic(double cell_size, int num_angle_bins,
                  double step_cost, double diff_angle_cost,double diff_depth_cost,double dist_cost,
                  double max_step_width, int Gridsearch_downsampling, double maxStepElevation)
  : Heuristic(cell_size, num_angle_bins, PATH_COST),
    ivpGrid(NULL),
    ivStepCost(step_cost),
    ivDiffAngleCost(diff_angle_cost),
    ivDiffDepthCost(diff_depth_cost),
    ivDistanceCost(dist_cost),
    ivMaxStepWidth(max_step_width),
    ivMaxStepElevation(maxStepElevation),
    ivGridsearch_downsampling(Gridsearch_downsampling),
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
 *    Make Sure that the dist is calculated optimally
 * **/
  // we devide by a 1000 to transform to somehow meters value
  double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(
      from_x, from_y)) / 1000.0;
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
  return (ivDistanceCost * dist + expected_steps * ivStepCost +
      diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost);
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
                            ivGoalX, ivGoalY,
                            from_x, from_y,
                            ivMaxStepElevation, sbpl_edit::SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }
  double d = (ros::Time::now() - start).toSec();
  ROS_INFO("Depth2DGridSearch Done in: %3.4lf seconds",d);

  // ! GRID2DSEARCH Publish
  int width = static_cast<int>(ivMapPtr->getInfo().width/ivGridsearch_downsampling);
  int height = static_cast<int>(ivMapPtr->getInfo().height/ivGridsearch_downsampling);
  double maximum = -1000;
  double minimum = 100000000;
  double max_cost = -1000000.0;
  cv::Mat distances = cv::Mat::zeros(width,height,CV_32F);
  for (int i=0;i<width;i++) {
    for(int j=0;j<height;j++){
      double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(
        ivGridsearch_downsampling*i, ivGridsearch_downsampling*j)) / 1000.0;

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

  // Update depth gridmap to be used by the gridsearch planner (downsampled)
  unsigned width = ivMapPtr->getInfo().width;
  unsigned height = ivMapPtr->getInfo().height;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();

  ivGridSearchPtr.reset(new sbpl_edit::DEPTH2DGridSearch(width, height,
                                             ivMapPtr->getResolution(), 4.0/65536.0,
                                              ivGridsearch_downsampling, ivMaxStepElevation) );
  width = ivMapPtr->getInfo().width/ivGridsearch_downsampling;
  height = ivMapPtr->getInfo().height/ivGridsearch_downsampling;
  ivpDepth2D = new float* [width];

  for (unsigned x = 0; x < width; ++x){
    ivpDepth2D[x] = new float [height];
  }
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      ivpDepth2D[x][y] = ivMapPtr->depthMapAtCell(x*ivGridsearch_downsampling,y*ivGridsearch_downsampling);
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

  unsigned width = model_output->width;
  unsigned height = model_output->height;

  if (ivGridSearchPtr)
    ivGridSearchPtr->destroy();

  ivGridSearchPtr.reset(new sbpl_edit::DEPTH2DGridSearch(width, height,
                                             ivMapPtr->getResolution(), 4.0/65536.0,
                                            ivGridsearch_downsampling, ivMaxStepElevation) );

  // Beginning To Fill the Grid Search Array from the Neural Network Output
  cv::Mat output = cv_ptr->image;
  ROS_INFO("Depth Footstep Planning Node Got Feasible Footsteps For the Map (Model Output). \n");

  // Update feasible footsteps gridmap to be used by the gridsearch planner (downsampled)
  // Initializing Gird array from Model Output for GridSearch
  width = ivMapPtr->getInfo().width/ivGridsearch_downsampling;
  height = ivMapPtr->getInfo().height/ivGridsearch_downsampling;

  ivpGrid = new unsigned char* [width];
  for (unsigned x = 0; x < width; ++x){
    ivpGrid[x] = new unsigned char [height];
  }
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      int i = y*ivGridsearch_downsampling, j = x*ivGridsearch_downsampling;
      int output_state = static_cast<int>(output.at<unsigned short>(i, j));
        for(int di=0; di<ivGridsearch_downsampling; di++)
            for(int dj=0; dj<ivGridsearch_downsampling; dj++)
                output_state = output_state >  static_cast<int>(output.at<unsigned short>(i+di, j+dj))?
                 static_cast<int>(output.at<unsigned short>(i+di, j+dj)): output_state;
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
  int width = ivMapPtr->getInfo().width/ivGridsearch_downsampling;
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
