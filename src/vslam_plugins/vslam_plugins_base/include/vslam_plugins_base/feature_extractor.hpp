#ifndef VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_

#include "vslam_datastructure/feature/base.hpp"

namespace vslam_feature_extractor_base {
  class FeatureExtractor {
  public:
    using Feature = vslam_datastructure::feature::Feature;
    using FeatureType = vslam_datastructure::feature::Feature::Type;
    using Features = std::vector<Feature>;

    virtual void initialize(int num_features) = 0;
    virtual Features extract_features(const cv::Mat& image) = 0;
    virtual FeatureType feature_type() = 0;
    virtual ~FeatureExtractor() {}

  protected:
    FeatureExtractor() {}

    FeatureType feature_type_{FeatureType::undefined};
  };
}  // namespace vslam_feature_extractor_base

#endif  // VSLAM_PLUGINS_BASE__FEATURE_EXTRACTOR_HPP_