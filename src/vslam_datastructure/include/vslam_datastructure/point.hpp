#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

#include <opencv2/opencv.hpp>

namespace vslam_datastructure {

  struct Point {
    enum class Type { undefined = 0, orb = 1 };

    cv::KeyPoint keypoint;

    cv::Mat descriptor;

    Type type{Type::undefined};

    long unsigned int id{point_count++};

    static long unsigned int point_count;
  };

  using Points = std::vector<Point>;

  struct MatchedPoint {
    Point point1;
    Point point2;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
