#include "vslam_feature_extraction_plugins/orb.hpp"

#include <cmath>

namespace vslam_feature_extraction_plugins {

  void Orb::initialize(int num_features) {
    num_features_ = num_features;
    feature_type_ = OrbFeature::type;
  }

}  // namespace vslam_feature_extraction_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_extraction_plugins::Orb,
                       vslam_feature_extraction_base::FeatureExtraction)
