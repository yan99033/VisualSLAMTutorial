#ifndef VSLAM_CAMERA_TRACKER_PLUGINS__OPENCV_CAMERA_TRACKER_HPP_
#define VSLAM_CAMERA_TRACKER_PLUGINS__OPENCV_CAMERA_TRACKER_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"

namespace vslam_camera_tracker_plugins {

  class OpenCVCameraTracker : public virtual vslam_camera_tracker::base::CameraTracker {
  public:
    ~OpenCVCameraTracker() { std::cerr << "Terminated OpenCVCameraTracker" << std::endl; }

    void initialize() override {}

    bool track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                           cv::Mat& T_2_1) override;
    bool track_camera_3d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                           cv::Mat& T_2_1) override;

    inline std::string getPluginName() override { return "vslam_camera_tracker_plugins::OpenCVCameraTracker"; }

  private:
    static constexpr const double find_ess_mat_prob_{0.999};
    static constexpr const double find_ess_mat_thresh_{1.0};

    static constexpr const int pnp_ransac_num_iters_{100};
    static constexpr const float pnp_reproj_err_thresh_{8.0};
    static constexpr const double pnp_confidence_{0.99};
    static constexpr const bool use_extrinsic_guess_{false};

    static constexpr const double inlier_ratio_{0.5};
  };
}  // namespace vslam_camera_tracker_plugins

#endif  // VSLAM_CAMERA_TRACKER_PLUGINS__OPENCV_CAMERA_TRACKER_HPP_
