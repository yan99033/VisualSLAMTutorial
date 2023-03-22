#ifndef VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_

#include "vslam_datastructure/point.hpp"

namespace vslam_feature_extractor_base {
  class FeatureExtractor {
  public:
    using Point = vslam_datastructure::Point;
    using PointType = Point::Type;
    using Points = std::vector<Point>;

    virtual void initialize(int num_features) = 0;
    virtual Points extract_features(const cv::Mat& image) = 0;
    virtual PointType point_type() = 0;
    virtual ~FeatureExtractor() {}

  protected:
    FeatureExtractor() {}

    PointType point_type_{PointType::undefined};
  };
}  // namespace vslam_feature_extractor_base

#endif  // VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_