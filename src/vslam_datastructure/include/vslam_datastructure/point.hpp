#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

namespace vslam_datastructure {

  struct Point {
    enum class Type { undefined = 0, orb = 1 };

    cv::KeyPoint keypoint;

    cv::Mat descriptor;

    Type type{Type::undefined};
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
