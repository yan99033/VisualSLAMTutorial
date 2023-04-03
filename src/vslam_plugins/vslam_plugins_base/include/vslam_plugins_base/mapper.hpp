#ifndef VSLAM_PLUGINS_BASE__MAPPER_HPP_
#define VSLAM_PLUGINS_BASE__MAPPER_HPP_

#include "vslam_datastructure/point.hpp"

namespace vslam_mapper_base {
  class Mapper {
  public:
    virtual void initialize(const cv::Mat& K) = 0;
    virtual vslam_datastructure::MapPoints map(vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& T_1_w,
                                               const cv::Mat& T_2_1, const bool normalize_depth = false)
        = 0;
    virtual ~Mapper() {}

  protected:
    Mapper() {}
  };
}  // namespace vslam_mapper_base

#endif  // VSLAM_PLUGINS_BASE__MAPPER_HPP_