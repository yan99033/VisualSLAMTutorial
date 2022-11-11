#ifndef __CAMERA_TRACKER_H__
#define __CAMERA_TRACKER_H__

#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>

#include "datastructure/frame.h"
namespace vslam_libs {
  namespace camera_tracker {

    class CameraTracker {
    public:
      CameraTracker() = default;

      // Camera intrinsics and feature matcher
      explicit CameraTracker(const cv::Mat& cam_mat, cv::DescriptorMatcher* matcher = nullptr);

      // wrapper for cv::recoverPose
      // https://docs.opencv.org/4.5.0/d9/d0c/group__calib3d.html#gadb7d2dfcc184c1d2f496d8639f4371c0
      int recoverPose(const datastructure::FramePtr frame1, const datastructure::FramePtr frame2,
                      cv::OutputArray R, cv::OutputArray t, cv::InputOutputArray mask);

    private:
      cv::Mat cam_mat;
      cv::DescriptorMatcher* matcher;
    };

    void getCorrespondences(const std::vector<cv::KeyPoint>& keypoints1,
                            const std::vector<cv::KeyPoint>& keypoints2,
                            const std::vector<cv::DMatch>& matches, const std::vector<char>& mask,
                            std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2);

    class CameraTrackerError : public std::runtime_error {
    public:
      CameraTrackerError(const std::string msg) : runtime_error(msg) {}
    };
  }  // namespace camera_tracker
}  // namespace vslam_libs

#endif  // __LOAD_FROM_FOLDER_H__