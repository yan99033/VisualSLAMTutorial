#ifndef VSLAM_FEATURE_EXTRACTOR_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_EXTRACTOR_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"

namespace vslam_feature_matcher_plugins {

  class Orb : public vslam_feature_matcher_base::FeatureMatcher {
  public:
    void initialize() override;

    MatchedPoints match_features(const FramePair& frame_pair) override;

  protected:
    int num_features_{1000};

  private:
    cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;
  };
}  // namespace vslam_feature_matcher_plugins

#endif