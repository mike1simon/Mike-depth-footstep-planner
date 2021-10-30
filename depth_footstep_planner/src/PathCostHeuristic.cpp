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
  //* could be removed after more testing (then use ...noBounds... again)
//  ivMapPtr->worldToMapNoBounds(cell_2_state(current.getX(), ivCellSize),
//                               cell_2_state(current.getY(), ivCellSize),
//                               from_x, from_y);
  ivMapPtr->worldToMap(cell_2_state(current.getX(), ivCellSize),
                               cell_2_state(current.getY(), ivCellSize),
                               from_x, from_y);

  unsigned int to_x;
  unsigned int to_y;
  //* could be removed after more testing (then use ...noBounds... again)
//  ivMapPtr->worldToMapNoBounds(cell_2_state(to.getX(), ivCellSize),
//                               cell_2_state(to.getY(), ivCellSize),
//                               to_x, to_y);
  ivMapPtr->worldToMap(cell_2_state(to.getX(), ivCellSize),
                               cell_2_state(to.getY(), ivCellSize),
                               to_x, to_y);

  // cast to unsigned int is safe since ivGoalX/ivGoalY are checked to be >= 0
  // if ((unsigned int)ivGoalX != to_x || (unsigned int)ivGoalY != to_y)
  // {
  //   ROS_ERROR("PathCostHeuristic::getHValue to a different value than "
  //             "precomputed, heuristic values will be wrong. You need to call "
  //             "calculateDistances() before!");
  // }
  // assert((unsigned int)ivGoalX == to_x && (unsigned int)ivGoalY == to_y);
//  no grid search needed 
/**
 *  TODO: 
 *    1. Make Sure that the dist is calculated optimally
 *  ?  2. Print the Costs and view them in grid manner
 *  ?  3. Change the hard coded 0.8 to a parameter.
 * **/
  // ! Debugging calculating the distance 
  // std::cout<< ".";
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
  // ! OLD Debugging
  //  ROS_ERROR("Step cost between a[x: %d , y: %d] and b[x: %d , y: %d] is dist: %lf ivStepCost: %lf es: %lf all: %lf with depth: %lf"
  //            ,from_x,from_y,to_x,to_y,dist,ivStepCost,
  //            expected_steps,
  //            (dist + expected_steps * ivStepCost + diff_angle * ivDiffAngleCost),
  //            (2*dist + expected_steps * ivStepCost +
  //                  diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost));

  return (ivDistanceCost*dist + expected_steps * ivStepCost +
      diff_angle * ivDiffAngleCost + diff_depth * ivDiffDepthCost);
  // ! OLD Debugging
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
  // ! Debugging time
  ros::Time start = ros::Time::now();
  if ((int)to_x != ivGoalX || (int)to_y != ivGoalY)
  {
    ivGoalX = to_x;
    ivGoalY = to_y;
      // ! Debugging 
    unsigned int max_hight;
    // ! Change This when changing the accurcy from 1000x1000 to 500x500 and use downsampling also
    ivGridSearchPtr->search(ivpGrid,ivpDepth2D, cvObstacleThreshold,
                            ivGoalX/2, ivGoalY/2, from_x/2, from_y/2,
                            max_hight, sbpl_edit::SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
    // ivGridSearchPtr->search(ivpGrid,ivpDepth2D, cvObstacleThreshold,
    //                         from_x/2, from_y/2, ivGoalX/2, ivGoalY/2,
    //                         max_hight, sbpl_edit::SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
  }
  // ! Debugging time
  double d = (ros::Time::now() - start).toSec();
  ROS_ERROR("DEPTH2DGRIDSEARCH took: %lf seconds",d);
  // ! Debugging TEST GRID2DSEARCH
  int width = static_cast<int>(ivMapPtr->getInfo().width/2);
  int height = static_cast<int>(ivMapPtr->getInfo().height/2);
  double maximum = -1000;
  double minimum = 100000000;
  double max_cost = -1000000.0;
  cv::Mat distances = cv::Mat::zeros(width,height,CV_32F);
  for (int i=0;i<width;i++) {
    for(int j=0;j<height;j++){
      double dist = double(ivGridSearchPtr->getlowerboundoncostfromstart_inmm(j, i)) / 1000.0;
//        distances.at<double>(j,i) = dist;

      if(dist < 1000000.0)
        distances.at<float>(j,i) = dist;
      else
        distances.at<float>(j,i) = -1.0;
      if(dist > maximum)
        maximum = dist;
      if(dist < minimum)
        minimum = dist;
      if(dist < 1000000.0 && dist > max_cost)
        max_cost = dist;
    }
  }
  ROS_ERROR("maximum = %lf  minimum = %lf max_cost = %lf  max = %d ",maximum,minimum,max_cost,ivGridSearchPtr->getlargestcomputedoptimalf_inmm());

  for (int i=0;i<width;i++) {
    for(int j=0;j<height;j++){
      if(distances.at<float>(i,j) < 0){
        distances.at<float>(i,j) = 1.0;
      }
      else
        distances.at<float>(i,j) = distances.at<float>(i,j)/(max_cost);
    }
  }

  cv::imshow("2DGRIDSEARCH", distances);
  cv::imwrite("/home/mike/thesis_ws/2DGRIDSEARCH.jpg",distances);
  cv::waitKey(3000);
  cv::destroyAllWindows();
  // ! Finished Debugging Code
  return true;
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
                                             ivMapPtr->getResolution(), 4.0/65536.0 ) );
  // ! Debugging getting Output 
  // ! instead of calculating the output using the neural network,
  // ! we are reading it right away for Debugging purposes 
  cv::Mat output = imread("/home/mike/thesis_ws/src/Mike-depth-footstep-planner/depthmap_humanoid_msgs/map_test/output/test2.png",-1);
  // cv::rotate(output,output,cv::ROTATE_90_CLOCKWISE);
  cv::Mat input = imread("/home/mike/thesis_ws/src/Mike-depth-footstep-planner/depthmap_humanoid_msgs/map_test/input/test2.png",-1);

  ROS_ERROR("loaded output size: %dx%d",output.rows,output.cols);

  // ! Debugging output type
  int type = output.type();
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');
  std::cout << "output type: " << r <<std::endl;

  // ! Debugging Initializing Gird array from output for GridSearch
  ivpGrid = new unsigned char* [width];
  ivpDepth2D = new float* [width];

  for (unsigned x = 0; x < width; ++x){
    ivpGrid[x] = new unsigned char [height];
    ivpDepth2D[x] = new float [height];
  }
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
//      float dist = 0;//ivMapPtr->distanceMapAtCell(x,y);
      // int dist = static_cast<int>(output.at<Vec3b>(x,y).val[0]);
      int dist = static_cast<int>(output.at<unsigned short>(x,y));
      if (dist < 0)
        ROS_ERROR("Distance map at %d %d dist: %f out of bounds", x, y,dist);
      else if (dist <= 100)
        ivpGrid[x][y] = 255;
      else
        ivpGrid[x][y] = 0;
//      ROS_ERROR("output map at %d %d dist: %d ivpGrid: %d", x, y,dist,static_cast<int>(ivpGrid[x][y]));
      ivpDepth2D[x][y] = ivMapPtr->depthMapAtCell(x*2,y*2);
    }
  }

  // ! Debugging Printing purposes
  cv::resize(ivMapPtr->depthMap()/4.0,input,cv::Size(width,height));
  cv::imshow("input",input);
  cv::imshow("Original Output", output);
  cv::waitKey(3000);
  cv::destroyAllWindows();

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
  // // ! Debugging getting Output 
  // // ! instead of calculating the output using the neural network,
  // // ! we are reading it right away for Debugging purposes 
  // // cv::Mat output = imread("/home/mike/thesis_ws/src/Mike-depth-footstep-planner/depthmap_humanoid_msgs/map_test/output/test2.png",-1);
  // // cv::rotate(output,output,cv::ROTATE_90_CLOCKWISE);
  // // cv::Mat input = imread("/home/mike/thesis_ws/src/Mike-depth-footstep-planner/depthmap_humanoid_msgs/map_test/input/test2.png",-1);

  // ? Beginning To Fill the Grid Search Array from the Neural Network Output
  cv::Mat output = cv_ptr->image;
  ROS_WARN("Got Model OUTPUT size: %dx%d",output.rows,output.cols);

  // ! Debugging output type
  int type = output.type();
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');
  std::cout << "Model output type: " << r <<std::endl;

  // // ! Debugging Initializing Gird array from output for GridSearch
  // ? Initializing Gird array from Model Output for GridSearch
  ivpGrid = new unsigned char* [width];
  for (unsigned x = 0; x < width; ++x){
    ivpGrid[x] = new unsigned char [height];
  }
  for (unsigned y = 0; y < height; ++y)
  {
    for (unsigned x = 0; x < width; ++x)
    {
      int output_state = static_cast<int>(output.at<unsigned short>(x,y));
      if (output_state < 0)
        ROS_ERROR("output_state of Model Output of map at %d %d is: %d out of bounds", x, y,output_state);
      else if (output_state <= 100)
        ivpGrid[x][y] = 255;
      else
        ivpGrid[x][y] = 0;
//      ROS_ERROR("output_state of Model Output of map at %d %d dist: %d ivpGrid: %d", x, y,dist,static_cast<int>(ivpGrid[x][y]));
    }
  }

  // ! Debugging Printing purposes
  cv::imshow("Model Output", output);
  cv::waitKey(3000);
  cv::destroyAllWindows();

}


void
PathCostHeuristic::resetGrid()
{
//   CvSize size = ivMapPtr->size(); // here we get (height; width) instead of (width; height)
  int width = ivMapPtr->getInfo().width/2;
//  ROS_ERROR("ivMapPtr->getInfo().width: %d  width: %d",ivMapPtr->getInfo().width,width);
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
//  std::cout << "entered" << std::endl;
//  std::cout << "entered" << ivMapPtr->getInfo().width<< std::endl;
//  std::cout << "entered" << std::endl;

}
} // end of namespace
