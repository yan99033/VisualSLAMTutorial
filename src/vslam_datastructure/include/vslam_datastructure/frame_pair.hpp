#ifndef VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {

  class FramePair {
  public:
    enum class ConstraintType { init = 0, tracking = 1, loop_closure = 2, relocalization = 3 };

    using Point = vslam_datastructure::Point;
    using PointType = vslam_datastructure::Point::Type;
    using Points = std::vector<Point>;
    using MatchedPoint = vslam_datastructure::MatchedPoint;
    using MatchedPoints = std::vector<MatchedPoint>;

    private:
    Frame* frame1_;
    Frame* frame2_;

    MatchedPoints matches_;

    // Camera pose
    Sophus::SE3d T_1_2;

    ConstraintType constraint_type_{ConstraintType::init};
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_