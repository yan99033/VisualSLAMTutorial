#include "monocular_camera_plugins/virtual_camera.hpp"

#include <cassert>
#include <filesystem>

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

    load_from_folder(image_folder);

    params_fs_.release();
  }

  cv::Mat VirtualCamera::grab_image() {
    // Stop at the end of the sequence
    if (i_ >= files_.size() - 1) {
      return cv::Mat();
    }

    cv::Mat image = cv::imread(files_.at(i_), cv::IMREAD_COLOR);
    image = undistorter_->undistort_image(image);

    i_++;

    return image;
  }

  void VirtualCamera::load_from_folder(const std::string &folder, const std::string &ext) {
    for (const auto &f : std::filesystem::directory_iterator(folder)) {
      if (f.path().extension() == ext) {
        files_.push_back(f.path());
      }
    }

    assert(!files_.empty());

    std::sort(files_.begin(), files_.end());
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::VirtualCamera, camera_plugins::base::MonocularCamera)