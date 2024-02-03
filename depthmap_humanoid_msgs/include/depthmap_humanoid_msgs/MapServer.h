#ifndef DEPTHMAP_HUMANOID_MSGS_MAP_SERVER_H
#define DEPTHMAP_HUMANOID_MSGS_MAP_SERVER_H

#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nav_msgs/MapMetaData.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/tf.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

#include "depthmap_humanoid_msgs/GetGreyScaleMap16bitMap.h"
#include "depthmap_humanoid_msgs/GreyScaleMap16bit.h"
#include "depthmap_humanoid_msgs/DepthMap.h"
#include "depthmap_humanoid_msgs/GetDepthMap.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <cmath>
#include "ros/ros.h"
#include "ros/console.h"
#include "yaml-cpp/yaml.h"


namespace map_server {
#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
class MapServer
{
public:
  MapServer(const std::string& fname, double res );

  cv::Mat loadMapFromYamlFile(const std::string& fname, double res , nav_msgs::MapMetaData& info);
  nav_msgs::OccupancyGrid turnMaptoOccupancyGrid(cv::Mat IMG, nav_msgs::MapMetaData info);
  sensor_msgs::ImagePtr turnMaptoGreyScaleMap16bit(cv::Mat IMG, nav_msgs::MapMetaData info);
  depthmap_humanoid_msgs::DepthMap turnMaptoDepthMap(cv::Mat IMG, nav_msgs::MapMetaData info);
  PointCloud::Ptr turnMaptoPointCloud(cv::Mat IMG,nav_msgs::MapMetaData info);

private:
  ros::NodeHandle nh;
  ros::Publisher metaData_pub;
  ros::Publisher map_8bit_pub;
  ros::Publisher map_16bit_pub;
  ros::Publisher depthmap_pub;
  ros::Publisher pointCloud_pub;
  ros::ServiceServer map_8bit_srv;
  ros::ServiceServer map_16bit_srv;
  ros::ServiceServer depthmap_srv;

  bool res_from_file;
  int skip_pixel_pointcloud_x=1,skip_pixel_pointcloud_y=1;
  double max_attitude=4.0,min_attitude=0.0;
  nav_msgs::MapMetaData metaData;
  cv::Mat mapImg;
  nav_msgs::OccupancyGrid grayScaleMap8;
  PointCloud::Ptr pointCloud;
  // depthmap_humanoid_msgs::GreyScaleMap16bit grayScaleMap16;
  sensor_msgs::ImagePtr grayScaleMap16;
  depthmap_humanoid_msgs::DepthMap depthmap;
  // sensor_msgs::ImagePtr depthmap;
  };

  }
#endif // DEPTHMAP_HUMANOID_MSGS_MAP_SERVER_H
