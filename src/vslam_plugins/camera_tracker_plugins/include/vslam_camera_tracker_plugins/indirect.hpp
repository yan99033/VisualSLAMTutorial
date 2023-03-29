#ifndef VSLAM_CAMERA_TRACKER_PLUGINS__INDIRECT_HPP_
#define VSLAM_CAMERA_TRACKER_PLUGINS__INDIRECT_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"

namespace vslam_camera_tracker_plugins {

  class Indirect : public vslam_camera_tracker_base::CameraTracker {
  public:
    void initialize(const cv::Mat& K) override;

    virtual cv::Mat track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points) override;

  protected:
    cv::Mat K;

  private:
    cv::Ptr<cv::ORB> orb_feature_detector_;
  };
}  // namespace vslam_camera_tracker_plugins

#endif  // VSLAM_CAMERA_TRACKER_PLUGINS__INDIRECT_HPP_
