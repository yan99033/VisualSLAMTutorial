#ifndef VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {

  class FramePair {
  public:
    enum class ConstraintType { init = 0, tracking = 1, loop_closure = 2, relocalization = 3 };

  private:
    Frame* frame1_;
    Frame* frame2_;

    MatchedPoints matches_;

    // Camera pose
    cv::Mat T_1_2;

    ConstraintType constraint_type_{ConstraintType::init};
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_