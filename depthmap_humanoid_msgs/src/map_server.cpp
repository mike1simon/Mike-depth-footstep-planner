#include <ros/ros.h>
#include "depthmap_humanoid_msgs/MapServer.h"
#include "depthmap_humanoid_msgs/depthmap2d.h"

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "OR USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server");
  ros::NodeHandle nh;
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);

  try
  {
    map_server::MapServer ms(fname, res);
    //depthmap2d::DepthMap2D map(ms.getDepthMap());
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }
}
