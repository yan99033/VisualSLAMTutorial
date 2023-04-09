#include "vslam_backend_plugins/indirect.hpp"

namespace vslam_backend_plugins {
  void Indirect::initialize() {}

  void Indirect::add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    assert(keyframe->is_keyframe());

    keyframes_[keyframe->id()] = keyframe;

    current_keyframe_ = keyframe;
  }

  void Indirect::remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    if (keyframes_.find(keyframe->id()) != keyframes_.end()) {
      keyframes_.erase(keyframe->id());
    }
  }

  vslam_datastructure::Frame::SharedPtr Indirect::get_current_keyframe() { return current_keyframe_; }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::Indirect, vslam_backend_base::Backend)