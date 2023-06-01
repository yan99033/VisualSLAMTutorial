#ifndef VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
#define VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_

#include <condition_variable>
#include <mutex>
#include <thread>

#include "vslam_backend_plugins/backend.hpp"
#include "vslam_datastructure/frame.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {
  class IndirectOptimizer : public vslam_backend_plugins::Optimizer {
  public:
    ~IndirectOptimizer();

    /// Indirect optimizer initializer
    void initialize() override;

    /// Add a new keyframe to back-end
    /**
     * /param keyframe a new keyframe
     */
    void add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) override;

    /// Remove a keyframe to back-end
    /**
     * /param keyframe an existing keyframe
     */
    void remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) override;

    /// Get a keyframe using the id. Return a nullptr if the keyframe cannot be found
    vslam_datastructure::Frame::SharedPtr get_keyframe(const long unsigned int id) const override;

    /// Add a loop constraint and run pose-graph optimization
    /**
     * /param kf_id_1 first keyframe
     * /param kf_id_2 second keyframe
     * /param T_1_2 relative transformation from the first to second keyframe
     * /param sim3_scale relative scale from the first to second keyframe
     */
    void add_loop_constraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2,
                             const double sim3_scale) override;

    /// Export all the keyframes to frame msgs to refresh the visualizer
    std::vector<vslam_msgs::msg::Frame> get_all_keyframe_msgs() const override;

  private:
    // All keyframes (can be looked up using their id)
    std::map<long unsigned int, vslam_datastructure::Frame::SharedPtr> keyframes_;

    // The keyframe that is closest to the current camera pose
    vslam_datastructure::Frame::SharedPtr current_keyframe_;

    // Use it for reading and writing keyframes
    mutable std::mutex keyframe_mutex_;

    /// For notifying the local BA thread to run Local BA
    std::condition_variable local_ba_condition_;

    /// Mutex to be used with local_ba_condition_
    std::mutex local_ba_mutex_;

    /// A flag to indicate if local BA is running. If it's running, we don't initiate another local BA
    std::atomic_bool run_local_ba_{false};

    /// Local BA thread. When a new keyframe is added, this thread is used to run local BA
    std::thread local_ba_thread_;

    /// A loop that is running in the local BA thread
    void local_ba_loop();

    /// Get the latest `num_core_kfs_` core keyframes and their map points
    /**
     * \return core keyframes and map points
     */
    /// @return
    std::pair<CoreKfsSet, CoreMpsSet> get_core_keyframes_mappoints();

    /// Number of core keyframes used in local BA
    size_t num_core_kfs_{5};

    /// A flag to indicate if the pose-graph optimization is running
    std::atomic_bool loop_optimization_running_{false};

    /// Remove outlier map points (excluding the core map points)
    void remove_outlier_mappoints();

    /// A boolean to stop the thread
    std::atomic_bool exit_thread_{false};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
