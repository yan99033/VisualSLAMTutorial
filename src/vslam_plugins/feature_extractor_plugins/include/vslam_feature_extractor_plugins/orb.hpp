#ifndef VSLAM_FEATURE_EXTRACTOR_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_EXTRACTOR_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"

namespace vslam_feature_extractor_plugins {

  class Orb : public vslam_feature_extractor_base::FeatureExtractor {
  public:
    void initialize(int num_features) override;

    vslam_datastructure::Points extract_features(const cv::Mat& image) override;

  protected:
    int num_features_{1000};

  private:
    // goodFeaturesToTrack
    double quality_level_{0.005};
    double min_dist_{10};

    // ORB feature detector
    int nlevels_{8};
    double scale_factor_{1.2};
    int harris_block_size_{7};
    int patch_size_{31};
    int desc_size_{32};
  };
}  // namespace vslam_feature_extractor_plugins

#endif
