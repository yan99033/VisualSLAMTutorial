#include "vslam_camera_tracker_plugins/indirect.hpp"

#include <cmath>
#include <iostream>

namespace vslam_camera_tracker_plugins {
  void Indirect::initialize() {}

  Sophus::SE3d Indirect::track_camera_2d2d(
      const vslam_datastructure::MatchedPoints& matched_points) {}

}  // namespace vslam_camera_tracker_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_camera_tracker_plugins::Indirect,
                       vslam_camera_tracker_base::CameraTracker)