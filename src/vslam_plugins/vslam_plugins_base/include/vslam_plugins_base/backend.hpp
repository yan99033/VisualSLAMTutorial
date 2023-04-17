#ifndef VSLAM_PLUGINS_BASE__BACKEND_HPP_
#define VSLAM_PLUGINS_BASE__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/frame_queue.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_backend_base {
  class Backend {
  public:
    virtual void initialize(const cv::Mat& K, const vslam_datastructure::FrameQueue::SharedPtr frame_queue = nullptr)
        = 0;

    virtual void add_keyframe(vslam_datastructure::Frame::SharedPtr frame) = 0;

    virtual void remove_keyframe(vslam_datastructure::Frame::SharedPtr frame) = 0;

    // The current keyframe could be the latest keyframe added to the backend or
    // a previous keyframe that is similar to the current frame (calculate by the place
    // recognition method)
    virtual vslam_datastructure::Frame::SharedPtr get_current_keyframe() = 0;

    virtual ~Backend() {}

  protected:
    Backend() {}

    // frame_queue_ is used to add the updated keyframes so they can be updated visually
    vslam_datastructure::FrameQueue::SharedPtr frame_queue_;
  };
}  // namespace vslam_backend_base

#endif  // VSLAM_PLUGINS_BASE__BACKEND_HPP_