#include "depthmap_humanoid_msgs/MapServer.h"

#define MAP_IDX(sx, x, y) ((sx) * (y) + (x))
#define invertYandSwitchXY(sx, x, y) ((sx) * (y) + (x))
#define map_indx(sx, x, y) ((sx) * (x) + (y))
#define invertY(sx,sy,x,y) ((sy) * (x) + (sy - y-1))

namespace map_server {
MapServer::MapServer(const std::string& fname, double res){

  ros::NodeHandle private_nh("~");

  mapImg = loadMapFromYamlFile(fname,res,metaData);
//  cv::namedWindow("test");
//  cv::imshow("test",mapImg);
//  cv::waitKey(0);

  grayScaleMap8 = turnMaptoOccupancyGrid(mapImg,metaData);
  grayScaleMap16 = turnMaptoGreyScaleMap16bit(mapImg,metaData);
  depthmap = turnMaptoDepthMap(mapImg,metaData);
  pointCloud = turnMaptoPointCloud(mapImg,metaData);
//  map_8bit_srv = nh.advertiseService("static_map", &MapServer::mapCallback, this);

  // Latched publisher for metadata
  metaData_pub= nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  metaData_pub.publish( metaData );

  // Latched publisher for data
  map_8bit_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_8bit_pub.publish( grayScaleMap8 );
  // map_16bit_pub = nh.advertise<depthmap_humanoid_msgs::GreyScaleMap16bit>("map16bit", 1, true);
  map_16bit_pub = nh.advertise<sensor_msgs::Image>("map16bit", 1, true);
  map_16bit_pub.publish( grayScaleMap16 );
  depthmap_pub = nh.advertise<depthmap_humanoid_msgs::DepthMap>("depthmap", 1, true);
  // depthmap_pub = nh.advertise<sensor_msgs::Image>("depthmap", 1, true);
  depthmap_pub.publish( depthmap );

  pointCloud_pub = nh.advertise<PointCloud>("depthmap_pointcloud", 1,true);
  pointCloud_pub.publish(pointCloud);



}
cv::Mat MapServer::loadMapFromYamlFile(const std::string& fname, double res, nav_msgs::MapMetaData& info){
  cv::Mat IMG;
  std::string mapfname = "";
  double origin[3];
  //mapfname = fname + ".pgm";
  //std::ifstream fin((fname + ".yaml").c_str());
  std::ifstream fin(fname.c_str());
  if (fin.fail()) {
    ROS_ERROR("Map_server could not open %s.", fname.c_str());
    exit(-1);
  }
#ifdef HAVE_YAMLCPP_GT_0_5_0
  // The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  YAML::Node doc;
  
  parser.GetNextDocument(doc);
#endif

  res_from_file = (res != 0.0);
    if (!res_from_file) {
      try {
        doc["resolution"] >> res;
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        exit(-1);
      }
    }

    try {
      doc["origin"][0] >> origin[0];
      doc["origin"][1] >> origin[1];
      doc["origin"][2] >> origin[2];
    } catch (YAML::InvalidScalar &) {
      ROS_ERROR("The map does not contain an origin tag or it is invalid.");
      exit(-1);
    }
    try {
      doc["skip_pixels_in_pointcloud"][0] >> skip_pixel_pointcloud_x;
      doc["skip_pixels_in_pointcloud"][1] >> skip_pixel_pointcloud_y;
    } catch (YAML::InvalidScalar &) {
      ROS_INFO("The map does not contain an skip_pixels_in_pointcloud tag or it is invalid setting it to 1.");
      skip_pixel_pointcloud_x = skip_pixel_pointcloud_y = 1;
    }
    try {
      doc["max_attitude"] >> max_attitude;
      doc["min_attitude"] >> min_attitude;
    } catch (YAML::InvalidScalar &) {
      ROS_INFO("The map does not contain an max_attitude or min_attitude tag or it is invalid setting it to 4.0 0.0.");
      max_attitude = 4.0;
      min_attitude = 0.0;
    }
  try {
    doc["image"] >> mapfname;
    if(mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      exit(-1);
    }

    boost::filesystem::path mapfpath(mapfname);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(fname);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      mapfname = mapfpath.string();
    }
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    exit(-1);
  }

  ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
  try
  {
//    IMG = cv::imread(mapfname,-1);

    IMG = cv::imread(mapfname, cv::IMREAD_ANYDEPTH);
    if(IMG.empty()){
      ROS_ERROR("error while reading the map.");
      exit(-1);
    }
    IMG.convertTo(IMG,CV_16UC1);

  }
  catch (std::runtime_error e)
  {
      ROS_ERROR("%s", e.what());
      exit(-1);
  }

  info.height = static_cast<uint32_t>(IMG.rows);
  info.width  = static_cast<uint32_t>(IMG.cols);
  info.resolution = static_cast<float>(res);
  info.origin.position.x = origin[0];
  info.origin.position.y = origin[1];
  tf::Quaternion q;
  q.setRPY(0.0,0.0,origin[2]);
  info.origin.orientation.x = q.x();
  info.origin.orientation.y = q.y();
  info.origin.orientation.z = q.z();
  info.origin.orientation.w = q.w();

  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
           info.width,
           info.height,
           res);

  return IMG;
}
nav_msgs::OccupancyGrid MapServer::turnMaptoOccupancyGrid(cv::Mat IMG, nav_msgs::MapMetaData info){
  nav_msgs::OccupancyGrid output;
  std::string frame_id;
  nh.param("frame_id", frame_id, std::string("map"));
  output.info = info;
  output.info.map_load_time = ros::Time::now();
  output.header.frame_id = frame_id;
  output.header.stamp = ros::Time::now();
  output.data.resize(info.width * info.height);

  for(int i=0; i<static_cast<int>(info.height) ; i++)
      for(int j=0; j<static_cast<int>(info.width); j++) {
        int raw_value = IMG.at<ushort>(i,j);
        int8_t greyValue = static_cast<int8_t>(100-(raw_value/(655)));
        // ! old mapping
        // int index = MAP_IDX(static_cast<int>(info.width),j,static_cast<int>(info.height) - i - 1);
        //int index = invertYandSwitchXY(static_cast<int>(info.height),i,static_cast<int>(info.width) - j - 1);
        //int index = map_indx(static_cast<int>(info.height),i,static_cast<int>(info.height) - j - 1);
        //int index = invertY(static_cast<int>(info.height),static_cast<int>(info.width),i,j);
        int index = i + j*info.height;
        output.data[static_cast<unsigned long>(index) ] = greyValue;
        //output.data[i*info.height+j] = greyValue;
      }
  return output;
}

// ! custom msg type deprecated
// depthmap_humanoid_msgs::GreyScaleMap16bit MapServer::turnMaptoGreyScaleMap16bit(cv::Mat IMG, nav_msgs::MapMetaData info){
sensor_msgs::ImagePtr MapServer::turnMaptoGreyScaleMap16bit(cv::Mat IMG, nav_msgs::MapMetaData info){

  std::string frame_id;
  nh.param("frame_id", frame_id, std::string("map"));
  
  // ! custom msg type deprecated
  // depthmap_humanoid_msgs::GreyScaleMap16bit output;
  // output.info = info;
  // output.info.map_load_time = ros::Time::now();
  // output.header.frame_id = frame_id;
  // output.header.stamp = ros::Time::now();

  cv_bridge::CvImage cv_image;
  cv_image.image = IMG;
  cv_image.header.frame_id   = frame_id; // Same tf frame as input image
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

  // ! custom msg type deprecated
  // output.data.resize(info.width * info.height);
  // output.maximum_attitude = static_cast<float>(max_attitude);
  // output.minimum_attitude = static_cast<float>(min_attitude);
  
  // for(int i=0; i<static_cast<int>(info.height) ; i++)
  //     for(int j=0; j<static_cast<int>(info.width); j++) {
  //       ushort raw_value = IMG.at<ushort>(i,j);
  //       //int8_t greyValue = static_cast<int8_t>(100-(x/(655)));
  //       int index = MAP_IDX(static_cast<int>(info.width),j,static_cast<int>(info.height) - i - 1);
  //       //int index = invertYandSwitchXY(static_cast<int>(info.height),i,static_cast<int>(info.width) - j - 1);
  //       //int index = map_indx(static_cast<int>(info.height),i,static_cast<int>(info.height) - j - 1);
  //       //int index = invertY(static_cast<int>(info.height),static_cast<int>(info.width),i,j);
  //       //output.data[static_cast<unsigned long>(index) ] = greyValue;
  //       //output.data[i*info.height+j] = greyValue;
  //       output.data[static_cast<unsigned long>(index)] = raw_value;
  //     }

  return cv_image.toImageMsg();

}

depthmap_humanoid_msgs::DepthMap MapServer::turnMaptoDepthMap(cv::Mat IMG, nav_msgs::MapMetaData info){
// sensor_msgs::ImagePtr MapServer::turnMaptoDepthMap(cv::Mat IMG, nav_msgs::MapMetaData info){
  std::string frame_id;
  nh.param("frame_id", frame_id, std::string("map"));
  
  depthmap_humanoid_msgs::DepthMap output;
  output.info = info;
  output.info.map_load_time = ros::Time::now();
  output.header.frame_id = frame_id;
  output.header.stamp = ros::Time::now();

  // output.data.resize(info.width * info.height);
  // for(int i=0; i<static_cast<int>(info.height) ; i++)
  //     for(int j=0; j<static_cast<int>(info.width); j++) {
  //       ushort raw_value = IMG.at<ushort>(i,j);
  //       //! this should be (max_attitude - min_attitude)/65536 + min_attitude
  //       double depth_value = raw_value*(max_attitude/65536.0)+min_attitude;
  //       //depth_value = static_cast<double>(round(1000 * static_cast<int>(depth_value)))  / 1000.0;
  //       int index = MAP_IDX(static_cast<int>(info.width),j,static_cast<int>(info.height) - i - 1);
  //       //int index = invertYandSwitchXY(static_cast<int>(info.height),i,static_cast<int>(info.width) - j - 1);
  //       //int index = map_indx(static_cast<int>(info.height),i,static_cast<int>(info.height) - j - 1);
  //       //int index = invertY(static_cast<int>(info.height),static_cast<int>(info.width),i,j);
  //       //output.data[static_cast<unsigned long>(index) ] = greyValue;
  //       //output.data[i*info.height+j] = greyValue;
  //       output.data[static_cast<unsigned long>(index)] = static_cast<float>(depth_value);
  //     }
  // return output;

  cv::Mat DepthMap = IMG.clone();
  DepthMap.convertTo(DepthMap, CV_32FC1);
  DepthMap = DepthMap * (max_attitude - min_attitude)/65536.0 + min_attitude;
  cv_bridge::CvImage cv_image;
  cv_image.image = DepthMap;
  cv_image.header.frame_id   = frame_id; // Same tf frame as input image
  cv_image.header.stamp = ros::Time::now();
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  output.map = *cv_image.toImageMsg();
  // ROS_ERROR("STEP: %ld , WIDTH: %ld , Hight: %ld",output.map.step,output.map.width,output.map.height);
  return output;
}

PointCloud::Ptr MapServer::turnMaptoPointCloud(cv::Mat IMG,nav_msgs::MapMetaData info){
  PointCloud::Ptr pc(new PointCloud);
  std::string frame_id;
  nh.param("frame_id", frame_id, std::string("map"));
  pc->header.frame_id = frame_id;
  pc->height = info.height/static_cast<unsigned int>(skip_pixel_pointcloud_y);
  pc->width = info.width/static_cast<unsigned int>(skip_pixel_pointcloud_x);
  //pc->resize(info.height*info.width);

  for(int i=0; i<static_cast<int>(info.height) ; i=i+skip_pixel_pointcloud_y)
      for(int j=0; j<static_cast<int>(info.width); j=j+skip_pixel_pointcloud_x) {
        if(i>static_cast<int>(info.height) || j>static_cast<int>(info.width))
          break;
        int raw_value = IMG.at<ushort>(i,j);
        unsigned char color_value = static_cast<unsigned char>(raw_value/256);
        double depth_value = raw_value*(max_attitude/65536.0)+min_attitude;
        // double y = static_cast<double>(info.resolution)*(static_cast<double>(info.height)-static_cast<double>(i) +0.5);
        // double x = static_cast<double>(info.resolution)*(static_cast<double>(j)+0.5);
        double x = static_cast<double>(info.resolution)*(static_cast<double>(i) +0.5);
        double y = static_cast<double>(info.resolution)*(static_cast<double>(j)+0.5);

        tf::Vector3 P_old(x,y,depth_value);
        tf::Quaternion q;
        q.setX(info.origin.orientation.x);
        q.setY(info.origin.orientation.y);
        q.setZ(info.origin.orientation.z);
        q.setW(info.origin.orientation.w);
        tf::Vector3 t(info.origin.position.x,info.origin.position.y,info.origin.position.z);
        tf::Transform T(q,t);
        tf::Vector3 P = T*P_old;
        pcl::PointXYZRGB p;
//        p._PointXYZRGB::x = static_cast<float>(x);
//        p._PointXYZRGB::y = static_cast<float>(y);
//        p._PointXYZRGB::z = static_cast<float>(depth_value);
        p._PointXYZRGB::x = static_cast<float>(P.x());
        p._PointXYZRGB::y = static_cast<float>(P.y());
        p._PointXYZRGB::z = static_cast<float>(P.z());
        p._PointXYZRGB::r = color_value;
        p._PointXYZRGB::b = color_value;
        p._PointXYZRGB::g = color_value;
        pc->points.push_back(p);
      }
  pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp);

  return pc;
}

}
