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

    // Get a keyframe using the id. Return a nullptr if the keyframe cannot be found
    vslam_datastructure::Frame::SharedPtr get_keyframe(const long unsigned int id) const override;

    void add_loop_constraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2,
                             const double sim3_scale) override;

  private:
    using CoreKfsSet = std::set<vslam_datastructure::Frame*>;
    using CoreMpsSet = std::set<vslam_datastructure::MapPoint*>;

    // The keyframe that is closest to the current camera pose
    vslam_datastructure::Frame::SharedPtr current_keyframe_;

    // Use it for reading and writing keyframes
    mutable std::mutex keyframe_mutex_;

    // Local BA
    std::condition_variable local_ba_condition_;
    std::atomic_bool run_local_ba_{false};
    std::mutex local_ba_mutex_;
    std::thread local_ba_thread_;
    void local_ba_loop();
    std::pair<CoreKfsSet, CoreMpsSet> get_core_keyframes_mappoints();
    void run_local_ba(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints);
    size_t num_core_kfs_{5};

    static constexpr const double huber_kernel_delta_{2.4477};
    static constexpr const double huber_kernel_delta_sq_{5.991};

    std::atomic_bool loop_optimization_running_{false};
    void run_pose_graph_optimization(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                     const cv::Mat& T_1_2, const double sim3_scale);

    std::atomic_bool exit_thread_{false};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
