// TODO: Licence

#ifndef VSLAM_BACKEND_PLUGINS__UTILS_HPP_
#define VSLAM_BACKEND_PLUGINS__UTILS_HPP_

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <opencv2/opencv.hpp>

namespace vslam_backend_plugins {
  namespace utils {
    g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose);

    g2o::Sim3 cvMatToSim3(const cv::Mat& pose, const double scale);

  }  // namespace utils
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__UTILS_HPP_