#include "feature_detector/feature_detector.h"

namespace vslam_libs {
  namespace feature_detector {
    OrbFeatureDetector::OrbFeatureDetector(int num_features) {
      detector = cv::ORB::create(num_features);
    }

    void OrbFeatureDetector::detectAndCompute(cv::InputArray img, cv::InputArray mask,
                                              std::vector<cv::KeyPoint>& keypoints,
                                              cv::OutputArray descriptors,
                                              bool useProvidedKeypoints) {
      detector->detectAndCompute(img, mask, keypoints, descriptors, useProvidedKeypoints);
    }

  }  // namespace feature_detector
}  // namespace vslam_libs
