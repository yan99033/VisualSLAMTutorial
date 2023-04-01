#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

#include <opencv2/opencv.hpp>

#include "vslam_datastructure/frame.hpp"

namespace vslam_datastructure {
  struct Point;
  using PointSharedPtr = std::shared_ptr<Point>;

  struct MapPoint {
    using SharedPtr = std::shared_ptr<MapPoint>;

    cv::Point3d pt_3d;

    /*
     * @brief: the corresponding points
     */
    std::vector<PointSharedPtr> projections;

    long unsigned int id{point_count++};

    static long unsigned int point_count;
  };
  using MapPoints = std::vector<MapPoint::SharedPtr>;

  struct Point {
    using SharedPtr = std::shared_ptr<Point>;

    enum class Type { undefined = 0, orb = 1 };

    cv::KeyPoint keypoint;

    cv::Mat descriptor;

    Type type{Type::undefined};

    MapPoint::SharedPtr mappoint;

    Frame::WeakPtr frame;

    long unsigned int id{point_count++};

    static long unsigned int point_count;
  };
  using Points = std::vector<Point::SharedPtr>;

  struct MatchedPoint {
    Point::SharedPtr point1;
    Point::SharedPtr point2;
  };
  using MatchedPoints = std::vector<MatchedPoint>;

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
