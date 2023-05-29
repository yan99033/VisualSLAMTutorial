#ifndef VSLAM_PLUGINS_BASE__VISUALIZER_HPP_
#define VSLAM_PLUGINS_BASE__VISUALIZER_HPP_

#include "vslam_msgs/msg/frame.hpp"

namespace vslam_visualizer {
  namespace base {
    class Visualizer {
    public:
      using FrameVec = std::vector<vslam_msgs::msg::Frame>;

      virtual void initialize() = 0;

      virtual void add_live_frame(const vslam_msgs::msg::Frame& frame_msg) = 0;

      virtual void add_keyframe(const vslam_msgs::msg::Frame& frame_msg) = 0;

      virtual void remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) = 0;

      virtual void replace_all_keyframes(const FrameVec& frame_msgs) = 0;

      virtual ~Visualizer() {}

    protected:
      Visualizer() {}
    };
  }  // namespace base
}  // namespace vslam_visualizer

#endif  // VSLAM_PLUGINS_BASE__VISUALIZER_HPP_