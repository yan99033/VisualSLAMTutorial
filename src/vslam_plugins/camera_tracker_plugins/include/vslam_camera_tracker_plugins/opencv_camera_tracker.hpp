/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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

    /// Camera tracker initializer
    void initialize() override {}

    /// Track the relative transformation (T_2_1) given a vector containing the 2D-2D correspondences between frame1
    /// and frame2
    /**
     * The tracking quality is indicated by the number of inlier points relative to the total amount of
     * correspondences provided
     *
     * \param[in] matched_points a vector containing the 2D-2D correspondences between frame1 and frame2
     * \param[out] T_2_1 the camera transformation from frame2 to frame1
     * \return a boolean indicating the tracking quality
     */
    bool trackCamera2d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                         cv::Mat& T_2_1) override;

    /// Track the relative transformation (T_2_1) given a vector containing the 3D-2D correspondences between frame1
    /// and frame2
    /**
     * The tracking quality is indicated by the number of inlier points relative to the total amount of
     * correspondences provided
     *
     * \param[in] matched_points a vector containing the 3D-2D correspondences between frame1 and frame2
     * \param[out] T_2_1 the camera transformation from frame2 to frame1
     * \return a boolean indicating the tracking quality
     */
    bool trackCamera3d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                         cv::Mat& T_2_1) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_camera_tracker_plugins::OpenCVCameraTracker"; }

  private:
    /// level of confidence (probability) that the estimated matrix is correct
    /**
     * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gad245d60e64d0c1270dbfd0520847bb87
     */
    static constexpr const double find_ess_mat_prob_{0.999};

    /// Maximum distance from a point to an epipolar line in pixels, beyond which the point is considered an outlier and
    /// is not used for computing the final fundamental matrix
    /**
     * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gad245d60e64d0c1270dbfd0520847bb87
     */
    static constexpr const double find_ess_mat_thresh_{1.0};

    /// Number of iterations
    /**
     * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e
     */
    static constexpr const int pnp_ransac_num_iters_{100};

    /// Maximum allowed distance between the observed and computed point projections to consider it an inlier
    /**
     * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e
     */
    static constexpr const float pnp_reproj_err_thresh_{5.0};

    /// Probability that the algorithm produces a useful result
    /**
     * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e
     */
    static constexpr const double pnp_confidence_{0.99};

    /// If true, uses the provided rvec and tvec values as initial approximations
    /**
     * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e
     */
    static constexpr const bool use_extrinsic_guess_{false};

    /// The num_inliers / num_matched_points ratio has to be greater than this to consider if the tracking good
    static constexpr const double inlier_ratio_{0.5};
  };
}  // namespace vslam_camera_tracker_plugins

#endif  // VSLAM_CAMERA_TRACKER_PLUGINS__OPENCV_CAMERA_TRACKER_HPP_
