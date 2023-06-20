/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "monocular_camera_plugins/calicam.hpp"

#include "vslam_utils/undistorter.hpp"

namespace monocular_camera_plugins {
  void CaliCam::initialize(const std::string& params_file) {
    params_fs_ = cv::FileStorage(params_file, cv::FileStorage::READ);
    if (!params_fs_.isOpened()) {
      throw std::runtime_error("failed to open " + params_file);
    }
    cv::Mat K;
    cv::Mat dist_coeffs;
    cv::Mat xi;
    cv::Mat R;
    double fov;
    int camera_id;
    int image_height_out;
    int image_width_out;

    params_fs_["K"] >> K;
    params_fs_["D"] >> dist_coeffs;
    params_fs_["fov"] >> fov;
    params_fs_["xi"] >> xi;
    params_fs_["R"] >> R;
    params_fs_["camera_id"] >> camera_id;
    params_fs_["image_height_in"] >> image_height_;
    params_fs_["image_width_in"] >> image_width_;
    params_fs_["image_height_out"] >> image_height_out;
    params_fs_["image_width_out"] >> image_width_out;

    params_fs_.release();

    openCamera(camera_id);

    double fov_rad = fov * CV_PI / 180.;
    double focal = image_height_out / 2. / tan(fov_rad / 2.);
    cv::Mat K_new = (cv::Mat_<double>(3, 3) << focal, 0., image_width_out / 2. - 0.5, 0., focal,
                     image_height_out / 2. - 0.5, 0., 0., 1.);

    undistorter_ = std::make_unique<vslam_utils::camera::CalicamUndistorter>(
        K, image_width_out, image_height_out, dist_coeffs, xi.at<double>(0, 0), R, K_new);
    K_ = K_new;
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::CaliCam, camera_plugins::base::MonocularCamera)