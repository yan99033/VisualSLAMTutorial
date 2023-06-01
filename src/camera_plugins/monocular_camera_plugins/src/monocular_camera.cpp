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

    open_camera(camera_id);

    cv::Mat K;
    params_fs_["K"] >> K;

    cv::Mat dist_coeffs;
    params_fs_["D"] >> dist_coeffs;

    undistorter_ = std::make_unique<vslam_utils::camera::Undistorter>(K, image_width_, image_height_, dist_coeffs);
    K_ = undistorter_->K();

    params_fs_.release();
  }

  cv::Mat MonocularCamera::grab_image() {
    cv::Mat image;

    video_capture_ >> image;
    image = undistorter_->undistort_image(image.clone());

    return image;
  }

  void MonocularCamera::open_camera(int camera_id) {
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