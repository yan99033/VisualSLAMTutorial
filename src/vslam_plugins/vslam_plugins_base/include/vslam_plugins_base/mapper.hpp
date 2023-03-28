#ifndef VSLAM_PLUGINS_BASE__MAPPER_HPP_
#define VSLAM_PLUGINS_BASE__MAPPER_HPP_

#include "vslam_datastructure/point.hpp"

namespace vslam_mapper_base {
  class Mapper {
  public:
    virtual void initialize(const cv::Mat& K) = 0;
    virtual void map(vslam_datastructure::MatchedPoints& matched_points) = 0;
    virtual ~Mapper() {}

  protected:
    Mapper() {}
  };
}  // namespace vslam_mapper_base

#endif  // VSLAM_PLUGINS_BASE__MAPPER_HPP_