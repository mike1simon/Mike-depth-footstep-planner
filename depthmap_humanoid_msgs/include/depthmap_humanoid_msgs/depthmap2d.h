#ifndef DEPTHMAP_HUMANOID_MSGS_DEPTHMAP2D_H
#define DEPTHMAP_HUMANOID_MSGS_DEPTHMAP2D_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/MapMetaData.h>
#include <opencv2/core/types_c.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "tf/LinearMath/Quaternion.h"

#include <cv_bridge/cv_bridge.h>
#include "depthmap_humanoid_msgs/DepthMap.h"

using namespace cv;
namespace depthmap2d{
  typedef boost::shared_ptr<DepthMap2D> DepthMap2DPtr;
  typedef boost::shared_ptr<const DepthMap2D> DepthMap2DConstPtr;
  /**
   * @brief Stores a nav_msgs::OccupancyGrid in a convenient opencv cv::Mat
   * as binary map (free: 255, occupied: 0) and as distance map (distance
   * to closest obstacle in meter).
   */
  class DepthMap2D {
    public:
      DepthMap2D();
      ///@brief Create from depthmap_humanoid_msgs::DepthMap
      DepthMap2D(const depthmap_humanoid_msgs::DepthMapConstPtr& depth_map);

      ///@brief Copy constructor, performs a deep copy of underlying data structures
      DepthMap2D(const DepthMap2D& other);
      virtual ~DepthMap2D();

      ///@brief Takes input map x-y coordinates and transforms them to world frame
      void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

      ///@brief Takes input world x-y coordinates and transforms them to map frame
      bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

      ///@brief Takes input map x-y coordinates and transforms them to world frame WITHOUT boundary checks
      void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;

      ///@brief Checks if a coordinate is covered by the map extent (same as worldToMap)
      bool inMapBounds(double wx, double wy) const;

    ///! not to be used
    //  /**
    //   * Inflate occupancy map by inflationRadius
    //   */
    //  void inflateMap(double inflationRaduis);

      ///@brief Returns distance (in m) between two map coordinates (indices)
      inline double worldDist(unsigned x1, unsigned y1, unsigned x2, unsigned y2){
        return DepthMap2D::worldDist(cv::Point(x1, y1), cv::Point(x2, y2));
      }

      inline double worldDist(const cv::Point& p1, const cv::Point& p2){
        return DepthMap2D::pointDist(p1, p2) * m_mapInfo.resolution;
      }

      ///@brief Returns Euclidean distance between two points
      static inline double pointDist(const cv::Point& p1, const cv::Point& p2){
        return sqrt(pointDist2(p1, p2));
      }

      ///@brief Returns squared distance between two points
      static inline double pointDist2(const cv::Point& p1, const cv::Point& p2){
        return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
      }
    /* not to be used
    /// not to be used
    //  /// Returns distance (in m) at world coordinates <wx,wy> in m; -1 if out of bounds!
    //  float distanceMapAt(double wx, double wy) const;

    //  /// Returns distance (in m) at map cell <mx, my> in m; -1 if out of bounds!
    //  float distanceMapAtCell(unsigned int mx, unsigned int my) const;

    //  /// Returns map value at world coordinates <wx, wy>; out of bounds will be returned as 0!
    //  uchar binaryMapAt(double wx, double wy) const;

    //  /// Returns map value at map cell <mx, my>; out of bounds will be returned as 0!
    //  uchar binaryMapAtCell(unsigned int mx, unsigned int my) const;

    //  /// Returns map value at map cell <mx, my>; out of bounds will be returned as 0!
    //  uchar& binaryMapAtCell(unsigned int mx, unsigned int my);
    /// not to be used
    */

      ///@brief Returns depth (in meter) at world coordinates <wx,wy> in m; -1000 if out of bounds.
      float depthMapAt(double wx, double wy) const;

      ///@brief Returns depth (in meter) at map cell <mx, my> in m; -1000 if out of bounds.
      float depthMapAtCell(unsigned int mx, unsigned int my) const;

      ///! will be implemented later or not
    //  /// @return true if map is occupied at world coordinate <wx, wy>. Out of bounds
    //  /// 		will be returned as occupied.
    //  bool isOccupiedAt(double wx, double wy) const;
      /// will be implemented later or not
    //  /// @return true if map is occupied at cell <mx, my>
    //  bool isOccupiedAtCell(unsigned int mx, unsigned int my) const;

      ///@brief Initialize map from a ROS depthmap_humanoid_msgs/DepthMap message
      void setMap(const depthmap_humanoid_msgs::DepthMapConstPtr& depth_map);

      ///@brief Converts back into a ROS depthmap_humanoid_msgs::DepthMap msg
      depthmap_humanoid_msgs::DepthMap toDepthMapMsg() const;

      ///@brief Initialize from an existing cv::Map. mapInfo (in particular resolution) remains the same!
      void setMap(const cv::Mat& depth_map);

      inline const nav_msgs::MapMetaData& getInfo() const {return m_mapInfo;}
      inline float getResolution() const {return m_mapInfo.resolution; }
      /// returns the tf frame ID of the map (usually "/map")
      inline const std::string getFrameID() const {return m_frameId;}

      /// @return the cv::Mat depth image.
      const cv::Mat& depthMap() const {return m_depthMap;}

      /// @return the size of the cv::Mat depth image. Note that x/y are swapped wrt. height/width
      //inline const CvSize size() const {return m_depthMap.size();}
      inline const cv::Size size() const {return m_depthMap.size();}


    protected:
      cv::Mat m_depthMap;   ///< depth map (in meter)
      nav_msgs::MapMetaData m_mapInfo;
      std::string m_frameId;	///< "map" frame where ROS OccupancyGrid originated from

  };

}
#endif // DEPTHMAP_HUMANOID_MSGS_DEPTHMAP2D_H
