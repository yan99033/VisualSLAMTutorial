#ifndef __FEATURE_DETECTOR_H__
#define __FEATURE_DETECTOR_H__

#include <opencv2/calib3d.hpp>

namespace vslam_libs {
  namespace feature_detector {

    class OrbFeatureDetector {
    public:
      OrbFeatureDetector(int num_features);

    private:
      cv::Ptr<cv::ORB> detector; 
    };


  } // namespace feature_detector
} // namespace vslam_libs


#endif // __FEATURE_DETECTOR_H__