#ifndef __FEATURE_DETECTOR_H__
#define __FEATURE_DETECTOR_H__

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace vslam_libs {
  namespace feature_detector {

    class OrbFeatureDetector {
    public:
      OrbFeatureDetector(int num_features);

      void detectAndCompute(cv::InputArray img, cv::InputArray mask,
                            std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors,
                            bool useProvidedKeypoints = false);

    private:
      cv::Ptr<cv::ORB> detector;
    };

  }  // namespace feature_detector
}  // namespace vslam_libs

#endif  // __FEATURE_DETECTOR_H__