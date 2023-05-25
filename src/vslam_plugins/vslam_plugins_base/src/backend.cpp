#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_base {
  std::vector<vslam_msgs::msg::Frame> Backend::get_all_keyframe_msgs() const {
    std::vector<vslam_msgs::msg::Frame> keyframe_msgs;
    for (const auto& [_, kf] : keyframes_) {
      vslam_msgs::msg::Frame keyframe_msg;
      kf->to_msg(&keyframe_msg, true);
      keyframe_msgs.push_back(keyframe_msg);
    }

    return keyframe_msgs;
  }
}  // namespace vslam_backend_base