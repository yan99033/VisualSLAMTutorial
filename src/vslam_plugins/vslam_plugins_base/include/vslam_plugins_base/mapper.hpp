#ifndef VSLAM_PLUGINS_BASE__MAPPER_HPP_
#define VSLAM_PLUGINS_BASE__MAPPER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_mapper {
  namespace base {
    class Mapper : public virtual vslam_plugin::base::Plugin {
    public:
      virtual void initialize() = 0;
      virtual vslam_datastructure::MapPoints map(vslam_datastructure::MatchedPoints& matched_points,
                                                 const cv::Mat& T_1_w, const cv::Mat& T_2_1, const cv::Mat& K)
          = 0;
      virtual ~Mapper() {}

    protected:
      Mapper() {}
    };
  }  // namespace base
}  // namespace vslam_mapper

#endif  // VSLAM_PLUGINS_BASE__MAPPER_HPP_