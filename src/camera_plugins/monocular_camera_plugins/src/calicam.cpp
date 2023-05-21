#include "monocular_camera_plugins/calicam.hpp"

#include "vslam_utils/undistorter.hpp"

namespace monocular_camera_plugins {
  void CaliCam::initialize(const std::string& params_file) {
    params_fs_ = cv::FileStorage(params_file, cv::FileStorage::READ);
    if (!params_fs_.isOpened()) {
      std::cout << "Failed to open ini parameters" << std::endl;
      throw std::runtime_error("failed to open " + params_file);
    }
    cv::Mat K;
    cv::Mat dist_coeffs;
    cv::Mat xi;
    cv::Mat R;
    double fov;
    int camera_id;

    params_fs_["kl"] >> K;
    params_fs_["Dl"] >> dist_coeffs;
    params_fs_["fov"] >> fov;
    params_fs_["xil"] >> xi;
    params_fs_["Rl"] >> R;
    params_fs_["image_height"] >> image_height_;
    params_fs_["image_width"] >> image_width_;
    params_fs_["camera_id"] >> camera_id;

    params_fs_.release();

    open_camera(camera_id);

    double fov_rad = fov * CV_PI / 180.;
    double focal = image_height_ / 2. / tan(fov_rad / 2.);
    cv::Mat K_new = (cv::Mat_<double>(3, 3) << focal, 0., image_width_ / 2. - 0.5, 0., focal, image_height_ / 2. - 0.5,
                     0., 0., 1.);

    undistorter_ = std::make_unique<vslam_utils::CalicamUndistorter>(K, image_width_, image_height_, dist_coeffs,
                                                                     xi.at<double>(0, 0), R, K_new);
    K_ = K_new;
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::CaliCam, camera_plugins::base::MonocularCamera)