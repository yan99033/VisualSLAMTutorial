#include "vslam_backend_plugins/utils.hpp"

namespace vslam_backend_plugins {
  namespace utils {
    g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose) {
      Eigen::Matrix3d R;
      R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
          pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
          pose.at<double>(2, 2);
      Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

      return g2o::SE3Quat(R, t);
    }

    g2o::Sim3 cvMatToSim3(const cv::Mat& pose, const double scale) {
      Eigen::Matrix3d R;
      R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
          pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
          pose.at<double>(2, 2);
      Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

      return g2o::Sim3(R, t, scale);
    }
  }  // namespace utils
}  // namespace vslam_backend_plugins