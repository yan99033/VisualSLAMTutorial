#ifndef VSLAM_PLUGINS_BASE__VISUALIZER_HPP_
#define VSLAM_PLUGINS_BASE__VISUALIZER_HPP_

#include "vslam_msgs/msg/frame.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_visualizer {
  namespace base {
    class Visualizer : public virtual vslam_plugin::base::Plugin {
    public:
      using FrameVec = std::vector<vslam_msgs::msg::Frame>;

      /// Visualizer initializer
      virtual void initialize() = 0;

      /// Add a live camera pose to the visualizer (which may not be a keyframe)
      /**
       * \param[in] frame_msg Frame message
       */
      virtual void addLiveFrame(const vslam_msgs::msg::Frame& frame_msg) = 0;

      /// Add a keyframe camera pose and its map points to the visualizer
      /**
       * \param[in] frame_msg Frame message
       */
      virtual void addKeyfame(const vslam_msgs::msg::Frame& frame_msg) = 0;

      /// Remove a keyframe camera pose and its map points to the visualizer
      /**
       * \param[in] frame_msg Frame message
       */
      virtual void removeKeyframe(const vslam_msgs::msg::Frame& frame_msg) = 0;

      /// Replace all keyframes and their map points in the visualizer
      /**
       * \param[in] frame_msgs a vector of type Frame message
       */
      virtual void replaceAllKeyframes(const FrameVec& frame_msgs) = 0;

      /// Destructor
      virtual ~Visualizer() {}

    protected:
      /// Constructor
      Visualizer() {}
    };
  }  // namespace base
}  // namespace vslam_visualizer

#endif  // VSLAM_PLUGINS_BASE__VISUALIZER_HPP_