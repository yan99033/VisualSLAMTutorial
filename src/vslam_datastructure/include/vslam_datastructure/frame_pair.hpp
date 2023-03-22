#ifndef VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_

#include "vslam_datastructure/frame.hpp"

namespace vslam_datastructure {

  class FramePair {
    enum class ConstraintType { init = 0, tracking = 1, loop_closure = 2, relocalization = 3 };

  private:
    Frame* frame1_;
    Frame* frame2_;

    ConstraintType constraint_type_{ConstraintType::init};
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__FRAME_PAIR_HPP_