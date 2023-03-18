#ifndef VSLAM_FEATURE_EXTRACTION_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_EXTRACTION_PLUGINS__ORB_HPP_

#include <cmath>

#include "vslam_datastructure/feature/orb.hpp"
#include "vslam_feature_extraction_plugins/feature_extraction_base.hpp"

namespace vslam_feature_extraction_plugins {
  class Orb : public vslam_feature_extraction_base::FeatureExtraction {
  public:
    using FeatureType = vslam_datastructure::feature::Feature::Type;
    using OrbFeature = vslam_datastructure::feature::Orb;

    void initialize(int num_features) override;

    Feature extract_features() override;

    FeatureType feature_type() override { return feature_type_; }

  protected:
    int num_features_{1000};
  };

}  // namespace vslam_feature_extraction_plugins

#endif
