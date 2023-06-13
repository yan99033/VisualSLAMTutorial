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

#include "monocular_camera_plugins/monocular_camera.hpp"

namespace monocular_camera_plugins {

  MonocularCamera::~MonocularCamera() {
    if (params_fs_.isOpened()) {
      params_fs_.release();
    }

    if (video_capture_.isOpened()) {
      video_capture_.release();
    }
  }

  void MonocularCamera::initialize(const std::string& params_file) {
    params_fs_ = cv::FileStorage(params_file, cv::FileStorage::READ);

    if (!params_fs_.isOpened()) {
      throw std::runtime_error("failed to open " + params_file);
    }

    int camera_id;
    params_fs_["camera_id"] >> camera_id;
    params_fs_["image_height"] >> image_height_;
    params_fs_["image_width"] >> image_width_;

    openCamera(camera_id);

    cv::Mat K;
    params_fs_["K"] >> K;

    cv::Mat dist_coeffs;
    params_fs_["D"] >> dist_coeffs;

    undistorter_ = std::make_unique<vslam_utils::camera::Undistorter>(K, image_width_, image_height_, dist_coeffs);
    K_ = undistorter_->K();

    params_fs_.release();
  }

  cv::Mat MonocularCamera::grabImage() {
    cv::Mat image;

    video_capture_ >> image;
    image = undistorter_->undistortImage(image.clone());

    return image;
  }

  void MonocularCamera::openCamera(int camera_id) {
    if (video_capture_.isOpened()) {
      return;
    }

    video_capture_.open(camera_id);
    video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    if (!video_capture_.isOpened()) {
      throw std::runtime_error("Cannot open camera " + std::to_string(camera_id));
    }
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::MonocularCamera, camera_plugins::base::MonocularCamera)