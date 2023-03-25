#ifndef VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_
#define VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_

#include <sophus/se3.hpp>

#include "vslam_datastructure/point.hpp"

namespace vslam_camera_tracker_base {
  class CameraTracker {
  public:
    virtual void initialize(const cv::Mat& K) = 0;
    virtual Sophus::SE3d track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points)
        = 0;
    virtual ~CameraTracker() {}

  protected:
    CameraTracker() {}
  };
}  // namespace vslam_camera_tracker_base

#endif  // VSLAM_PLUGINS_BASE__CAMERA_TRACKER_HPP_