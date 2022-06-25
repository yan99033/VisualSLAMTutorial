#include "feature_detector/feature_detector.h"

namespace vslam_libs {
  namespace feature_detector {
    OrbFeatureDetector::OrbFeatureDetector(int num_features) {
      detector = cv::ORB::create(num_features);
    }

  } // namespace feature_detector
} // namespace vslam_libs

