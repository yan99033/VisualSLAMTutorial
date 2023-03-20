#ifndef VSLAM_FEATURE_EXTRACTOR_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_EXTRACTOR_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/feature/orb.hpp"
// #include "vslam_feature_extractor_plugins/visibility_control.h"
#include "vslam_plugins_base/feature_extractor.hpp"

namespace vslam_feature_extractor_plugins {

  class VSlamFeatureExtractorPlugins {
  public:
    VSlamFeatureExtractorPlugins();

    virtual ~VSlamFeatureExtractorPlugins();
  };

}  // namespace vslam_feature_extractor_plugins

#endif
