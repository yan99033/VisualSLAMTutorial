#ifndef VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_
#define VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_

// #include <sophus/se3.hpp>

#include "vslam_datastructure/point.hpp"

namespace vslam_camera_tracker_base {
  class CameraTracker {
  public:
    virtual void initialize(const cv::Mat& K) = 0;
    virtual cv::Mat track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points) = 0;
    virtual cv::Mat track_camera_3d2d(const vslam_datastructure::MatchedPoints& matched_points,
                                      cv::Mat T_2_1_init = cv::Mat())
        = 0;
    virtual ~CameraTracker() {}

  protected:
    CameraTracker() {}
  };
}  // namespace vslam_camera_tracker_base

#endif  // VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_