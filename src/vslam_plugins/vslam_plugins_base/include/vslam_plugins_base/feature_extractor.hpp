#ifndef VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_

#include "vslam_datastructure/point.hpp"

namespace vslam_feature_extractor {
  namespace base {
    class FeatureExtractor {
    public:
      virtual void initialize(int num_features) = 0;
      virtual vslam_datastructure::Points extract_features(const cv::Mat& image) = 0;
      vslam_datastructure::Point::Type point_type() const { return point_type_; };
      virtual ~FeatureExtractor() {}

    protected:
      FeatureExtractor() {}

      vslam_datastructure::Point::Type point_type_{vslam_datastructure::Point::Type::undefined};
    };
  }  // namespace base
}  // namespace vslam_feature_extractor

#endif  // VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_