#include "depthmap_humanoid_msgs/depthmap2d.h"

namespace depthmap2d {

DepthMap2D::DepthMap2D()
: m_frameId("/map")
{

}

DepthMap2D::DepthMap2D(const depthmap_humanoid_msgs::DepthMapConstPtr& depth_map) {

  DepthMap2D::setMap(depth_map);
//    cv::namedWindow("test");
//    cv::imshow("test",m_depthMap);
//    cv::waitKey(0);

}

DepthMap2D::DepthMap2D(const DepthMap2D& other)
 : m_depthMap(other.m_depthMap.clone()),
   m_mapInfo(other.m_mapInfo),
   m_frameId(other.m_frameId)
{

}

DepthMap2D::~DepthMap2D() {

}

void DepthMap2D::setMap(const depthmap_humanoid_msgs::DepthMapConstPtr& depth_map){
  m_mapInfo = depth_map->info;
  m_frameId = depth_map->header.frame_id;
  // allocate map structs so that x/y in the world correspond to x/y in the image
  // (=> cv::Mat is rotated by 90 deg, because it's row-major!)
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // copy the sensor_msgs/Image msg with this encoding to the cv_bridge pointer
    // cv_ptr = cv_bridge::toCvCopy(depth_map->map, depth_map->map.encoding);
    //// not reading correctly fix this ASAP
    cv_ptr = cv_bridge::toCvCopy(depth_map->map, sensor_msgs::image_encodings::TYPE_32FC1);
    m_depthMap = cv_ptr->image;
    // if it was empty and there was an error (exit)
    if(m_depthMap.empty()){
      ROS_ERROR("NO MAP ERROR");
      return;
    }
    ROS_INFO("DepthMap2D created with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);
  }
  catch (cv_bridge::Exception& e)
  {
    // if there was an error while reading the msg image print error and exit
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  // ! depricated

  // m_depthMap = cv::Mat(m_mapInfo.width,m_mapInfo.height,CV_32FC1);

  // std::vector<float>::const_iterator mapDataIter = depth_map->data.begin();
  // // iterate over map, store in image
  // // (0,0) is lower left corner of DepthMap
  // for(unsigned int j = 0; j < m_mapInfo.height; ++j){
  //   for(unsigned int i = 0; i < m_mapInfo.width; ++i){
  //     m_depthMap.at<float>(i,j) = *mapDataIter;
  //     ++mapDataIter;
  //   }
  // }

}

depthmap_humanoid_msgs::DepthMap DepthMap2D::toDepthMapMsg() const{
  depthmap_humanoid_msgs::DepthMap msg;
  msg.header.frame_id = m_frameId;
  msg.header.stamp = ros::Time::now();
  msg.info = m_mapInfo;
  msg.map.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  
  //! depricated
  // msg.data.resize(msg.info.height*msg.info.width);
  // // iterate over map, store in data
  // std::vector<float>::iterator mapDataIter = msg.data.begin();
  // // (0,0) is lower left corner of OccupancyGrid
  // for(unsigned int j = 0; j < m_mapInfo.height; ++j){
  //   for(unsigned int i = 0; i < m_mapInfo.width; ++i){
  //     *mapDataIter = m_depthMap.at<float>(i,j);
  //     ++mapDataIter;
  //   }
  // }
  cv_bridge::CvImage cv_image;
  cv_image.image = m_depthMap;
  cv_image.header.frame_id   = m_frameId; // Same tf frame as input image
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

  msg.map =  *cv_image.toImageMsg();

  return msg;
}

void DepthMap2D::setMap(const cv::Mat& depth_map){
  m_depthMap = depth_map.clone();

  ROS_INFO("DepthMap2D copied from existing cv::Mat with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);

}

/* not to be used
/// not to be used
//void DepthMap2D::inflateMap(double inflationRadius){
//  m_binaryMap = (m_distMap > inflationRadius );
//  // recompute distance map with new binary map:
//  //cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
//  cv::distanceTransform(m_binaryMap, m_distMap, DIST_L2, DIST_MASK_PRECISE);
//  m_distMap = m_distMap * m_mapInfo.resolution;
//}
///
*/


//// See costmap2D for mapToWorld / worldToMap implementations:

void DepthMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
//  wx = m_mapInfo.origin.position.x + (mx+0.5) * m_mapInfo.resolution;
//  wy = m_mapInfo.origin.position.y + (my+0.5) * m_mapInfo.resolution;
  double x = (mx+0.5) * static_cast<double>(m_mapInfo.resolution);
  double y = (my+0.5) * static_cast<double>(m_mapInfo.resolution);
  tf::Vector3 P_old(x,y,0);
  tf::Quaternion q;
  q.setX(m_mapInfo.origin.orientation.x);
  q.setY(m_mapInfo.origin.orientation.y);
  q.setZ(m_mapInfo.origin.orientation.z);
  q.setW(m_mapInfo.origin.orientation.w);
  tf::Vector3 t(m_mapInfo.origin.position.x,m_mapInfo.origin.position.y,m_mapInfo.origin.position.z);
  tf::Transform T(q,t);
  tf::Vector3 P = T*P_old;
  wx = P.x();
  wy = P.y();
}

void DepthMap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const {
//  mx = (int) ((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
//  my = (int) ((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);
  tf::Quaternion q;
  q.setX(m_mapInfo.origin.orientation.x);
  q.setY(m_mapInfo.origin.orientation.y);
  q.setZ(m_mapInfo.origin.orientation.z);
  q.setW(m_mapInfo.origin.orientation.w);
  tf::Quaternion q_inverse = q.inverse();
  tf::Vector3 World_rotated(wx,wy,0);
  tf::Transform T(q_inverse);
  tf::Vector3 World_unRotated = T*World_rotated;

  mx = static_cast<int>( ((World_unRotated.x() - m_mapInfo.origin.position.x) / static_cast<double>(m_mapInfo.resolution)) );
  my = static_cast<int>( ((World_unRotated.y() - m_mapInfo.origin.position.y) / static_cast<double>(m_mapInfo.resolution)) );

}

bool DepthMap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
  int Mx =-100,My=-100;
  DepthMap2D::worldToMapNoBounds(wx,wy,Mx,My);

  if(Mx >= 0 && Mx < m_mapInfo.width && My >= 0 && My < m_mapInfo.height){
    mx = static_cast<unsigned int>(Mx);
    my = static_cast<unsigned int>(My);
    return true;
  }

  return false;
}

bool DepthMap2D::inMapBounds(double wx, double wy) const{
  unsigned mx, my;
  return DepthMap2D::worldToMap(wx,wy,mx,my);
}

/* not to be used
//float DepthMap2D::distanceMapAt(double wx, double wy) const{
//  unsigned mx, my;

//  if (worldToMap(wx, wy, mx, my))
//    return m_distMap.at<float>(mx, my);
//  else
//    return -1.0f;
//}


//uchar DepthMap2D::binaryMapAt(double wx, double wy) const{
//  unsigned mx, my;

//  if (worldToMap(wx, wy, mx, my))
//    return m_binaryMap.at<uchar>(mx, my);
//  else
//    return 0;
//}

//float DepthMap2D::distanceMapAtCell(unsigned int mx, unsigned int my) const{
//  return m_distMap.at<float>(mx, my);
//}


//uchar DepthMap2D::binaryMapAtCell(unsigned int mx, unsigned int my) const{
//  return m_binaryMap.at<uchar>(mx, my);
//}

//uchar& DepthMap2D::binaryMapAtCell(unsigned int mx, unsigned int my){
//  return m_binaryMap.at<uchar>(mx, my);
//}

*/

float DepthMap2D::depthMapAt(double wx, double wy) const{
  unsigned mx, my;

  if (DepthMap2D::worldToMap(wx, wy, mx, my))
    return m_depthMap.at<float>(mx, my);
  else
    return -1000.0f;
}

float DepthMap2D::depthMapAtCell(unsigned int mx, unsigned int my) const{
  return m_depthMap.at<float>(mx, my);
}

/// will be implemented later or not
//bool DepthMap2D::isOccupiedAtCell(unsigned int mx, unsigned int my) const{
//  return (m_binaryMap.at<uchar>(mx, my) < 255);
//}

/// will be implemented later or not
//bool DepthMap2D::isOccupiedAt(double wx, double wy) const{
//  unsigned mx, my;
//  if (worldToMap(wx, wy, mx, my))
//    return isOccupiedAtCell(mx, my);
//  else
//    return true;
//}
}
