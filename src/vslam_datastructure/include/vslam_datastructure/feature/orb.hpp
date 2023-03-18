#ifndef VSLAM_DATASTRUCTURE__FEATURE__ORB_HPP_
#define VSLAM_DATASTRUCTURE__FEATURE__ORB_HPP_

#include <opencv2/core/types.hpp>

#include "vslam_datastructure/feature/base.hpp"

namespace vslam_datastructure {

  namespace feature {

    struct Orb : public Feature {
      static constexpr int desc_len = 32;
      static constexpr Type type{Type::orb};

      cv::KeyPoint keypoint;
      uint8_t descriptor[desc_len];
    };
  }  // namespace feature

}  // namespace vslam_datastructure

#endif
