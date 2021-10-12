#include "depth_footstep_planner/helper.h"

namespace depth_footstep_planner {


bool
pointWithinPolygon(int x, int y, const std::vector<std::pair<int, int> >& edges)
{
  int cn = 0;

  // loop through all edges of the polygon
  for(unsigned int i = 0; i < edges.size() - 1; ++i)
  {
    if ((edges[i].second <= y && edges[i + 1].second > y) ||
        (edges[i].second > y && edges[i + 1].second <= y))
    {
      float vt = (float)(y - edges[i].second) /
        (edges[i + 1].second - edges[i].second);
      if (x < edges[i].first + vt * (edges[i + 1].first - edges[i].first))
      {
        ++cn;
      }
    }
  }
  return cn & 1;
}


bool check_validity_stable_depth(int x, int y, int theta,
                                 int foot_height, int foot_width
                                 ,cv::Mat Map, double &highest_hight){

  int radius = sqrt(foot_height*foot_height+foot_width*foot_width)/2+1;
  highest_hight = -1e9;
  std::map<double,int> foot_points;
  for(int i=-radius;i<=radius;i++)
    for(int j=-radius;j<=radius;j++)
    if(x+i>=0 && x+i<Map.rows && y+j>=0 && y+j<Map.cols)
    {
      //double v = Map.at<float>((int)y+j,(int)x+i);

      double v = Map.at<float>((int)x+i,(int)y+j);
      highest_hight = max(highest_hight,v);
      foot_points[v]++;
    }
//  ROS_ERROR(" validity check in x:%d y:%d theta: %d  is: %d new depth: %f no of points: %d  max allowns: %d radius: %d ",x,y,theta,
//            (foot_points[highest_hight] > 0.85*(0.85*(2*radius+1)*(2*radius+1)))
//           ,highest_hight,foot_points[highest_hight],
//           (int)(0.85*(2*radius+1)*(2*radius+1)),radius);
  return foot_points[highest_hight] > 0.85*(0.85*(2*radius+1)*(2*radius+1));
}
double get_depth(int x, int y,cv::Mat Map){
  return double(Map.at<float>(x,y));
}


bool collision_check(int x, int y, int theta, double& new_depth,
                     int foot_height, int foot_width, int method,
                     const depthmap2d::DepthMap2D& depth_map)
{

  if(method == 0){
    bool valid = check_validity_stable_depth(x,y,theta,
                                foot_height,foot_width,
                            depth_map.depthMap(), new_depth);
    return !valid;
  }else if(method == 10){
    new_depth = get_depth(x,y,depth_map.depthMap());
    return false;
  }
  // need to implement other steps
  else {
    ROS_ERROR("NO METHOD FOR collision_check SELECTED");
    return true;}
}

}
