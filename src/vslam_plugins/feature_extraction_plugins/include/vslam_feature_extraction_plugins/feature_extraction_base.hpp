#ifndef VSLAM_FEATURE_EXTRACTION_PLUGINS__PLUGIN_BASE_HPP_
#define VSLAM_FEATURE_EXTRACTION_PLUGINS__PLUGIN_BASE_HPP_

#include "vslam_datastructure/feature/base.hpp"

namespace vslam_feature_extraction_base {
  class FeatureExtraction {
  public:
    using Feature = vslam_datastructure::feature::Feature;
    using FeatureType = vslam_datastructure::feature::Feature::Type;

    virtual void initialize(int num_features) = 0;
    virtual Feature extract_features() = 0;
    virtual FeatureType feature_type() = 0;
    virtual ~FeatureExtraction() {}

  protected:
    FeatureExtraction() {}

    FeatureType feature_type_{FeatureType::undefined};
  };
}  // namespace vslam_feature_extraction_base

#endif  // VSLAM_FEATURE_EXTRACTION_PLUGINS__PLUGIN_BASE_HPP_