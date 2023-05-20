#include "monocular_camera_plugins/usb_cam.hpp"

namespace monocular_camera_plugins {
  UsbCamera::~UsbCamera() { video_capture_.release(); }

  void UsbCamera::initialize(const std::string& params_file) {
    cv::FileStorage fs(params_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cout << "Failed to open ini parameters" << std::endl;
      throw std::runtime_error("failed to open " + params_file);
    }

    fs["K"] >> K_;
    fs["image_height"] >> image_height_;
    fs["image_width"] >> image_width_;
    fs["camera_id"] >> camera_id_;

    fs.release();

    if (image_height_ <= 0) {
      throw std::runtime_error("invalid image height (" + std::to_string(image_height_) + ")");
    }

    if (image_width_ <= 0) {
      throw std::runtime_error("invalid image width (" + std::to_string(image_width_) + ")");
    }

    video_capture_.open(camera_id_);
    video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    if (!video_capture_.isOpened()) {
      throw std::runtime_error("Cannot open camera " + std::to_string(camera_id_));
    }
  }

  cv::Mat UsbCamera::grab_image() {
    cv::Mat image;
    video_capture_ >> image;

    return image;
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::UsbCamera, camera_plugins::base::MonocularCamera)