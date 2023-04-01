#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

#include <opencv2/opencv.hpp>

#include "vslam_datastructure/frame.hpp"

namespace vslam_datastructure {
  struct MapPoint;
  using MapPointPtr = std::shared_ptr<MapPoint>;
  using MapPoints = std::vector<MapPointPtr>;

  struct Point {
    enum class Type { undefined = 0, orb = 1 };

    cv::KeyPoint keypoint;

    cv::Mat descriptor;

    Type type{Type::undefined};

    MapPointPtr mappoint;

    FramePtr frame;

    long unsigned int id{point_count++};

    static long unsigned int point_count;
  };

  using PointPtr = std::shared_ptr<Point>;
  using Points = std::vector<PointPtr>;
  using PointType = Point::Type;

  struct MatchedPoint {
    PointPtr point1;
    PointPtr point2;
  };

  using MatchedPoints = std::vector<MatchedPoint>;

  struct MapPoint {
    cv::Point3d pt_3d;

    /*
     * @brief: the corresponding points
     */
    std::vector<PointPtr> projections;

    long unsigned int id{point_count++};

    static long unsigned int point_count;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
