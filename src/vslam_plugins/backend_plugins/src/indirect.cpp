#include "vslam_backend_plugins/indirect.hpp"

namespace vslam_backend_plugins {
  void Indirect::initialize() {}

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::Indirect, vslam_backend_base::Backend)