#include <ros/ros.h>
#include "sbpl_edit/Depth2Dgridsearch.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  sbpl_edit::DEPTH2DGridSearch C(10,10,0.1f,0.01f);
//  DEPTH2DGridSearch c(10,10,0.1f);
  ROS_INFO("Hello world!");
}
