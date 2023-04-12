#ifndef VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
#define VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_

#include <condition_variable>
#include <mutex>
#include <thread>

#include "vslam_datastructure/frame.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {

  class Indirect : public vslam_backend_base::Backend {
  public:
    ~Indirect();

    void initialize() override;

    void add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) override;

    void remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) override;

    vslam_datastructure::Frame::SharedPtr get_current_keyframe() override;

  private:
    using CoreKfs = std::set<vslam_datastructure::Frame::SharedPtr>;
    using CoreMps = std::set<vslam_datastructure::MapPoint::SharedPtr>;

    // All keyframes (can be looked up using their id)
    std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr> keyframes_;

    // The keyframe that is closest to the current camera pose
    vslam_datastructure::Frame::SharedPtr current_keyframe_;

    // Local BA
    std::condition_variable local_ba_condition_;
    std::atomic_bool run_local_ba_{false};
    std::mutex local_ba_mutex_;
    std::thread local_ba_thread_;
    void local_ba_loop();
    std::pair<CoreKfs, CoreMps> get_core_keyframes_mappoints();
    void run_local_ba(CoreKfs& core_keyframes, CoreMps& core_mappoints);
    size_t num_core_kfs_{5};

    std::atomic_bool exit_thread_{false};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
