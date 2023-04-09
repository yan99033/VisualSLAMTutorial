#ifndef VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
#define VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {

  class Indirect : public vslam_backend_base::Backend {
  public:
    void initialize() override;

    void add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) override;

    vslam_datastructure::Frame::SharedPtr get_current_keyframe() override;

  private:
    std::unordered_map<long unsigned int, vslam_datastructure::Frame::SharedPtr> keyframes_;

    vslam_datastructure::Frame::SharedPtr current_keyframe_;
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
