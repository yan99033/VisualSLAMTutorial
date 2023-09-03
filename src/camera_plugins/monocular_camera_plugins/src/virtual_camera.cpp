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

#include "monocular_camera_plugins/virtual_camera.hpp"

#include <cassert>

#include "vslam_utils/io.hpp"

namespace monocular_camera_plugins {
  void VirtualCamera::initialize(const std::string &params_file) {
    params_fs_ = cv::FileStorage(params_file, cv::FileStorage::READ);
    if (!params_fs_.isOpened()) {
      throw std::runtime_error("failed to open " + params_file);
    }

    params_fs_["image_height"] >> image_height_;
    params_fs_["image_width"] >> image_width_;

    cv::Mat K;
    params_fs_["K"] >> K;

    cv::Mat dist_coeffs;
    params_fs_["D"] >> dist_coeffs;

    undistorter_ = std::make_unique<vslam_utils::camera::Undistorter>(K, image_width_, image_height_, dist_coeffs);
    K_ = undistorter_->K();

    std::string image_folder;
    params_fs_["image_folder"] >> image_folder;

    files_ = vslam_utils::io::loadFromFolder(image_folder);

    if (files_.empty()) {
      throw std::runtime_error("No image files found in " + image_folder);
    }

    params_fs_.release();
  }

  cv::Mat VirtualCamera::grabImage() {
    // Stop at the end of the sequence
    if (i_ >= files_.size() - 1) {
      return cv::Mat();
    }

    cv::Mat image = cv::imread(files_.at(i_), cv::IMREAD_COLOR);
    image = undistorter_->undistortImage(image);

    i_++;

    return image;
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::VirtualCamera, camera_plugins::base::MonocularCamera)