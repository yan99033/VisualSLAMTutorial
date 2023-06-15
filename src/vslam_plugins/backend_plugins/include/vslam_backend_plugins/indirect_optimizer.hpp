/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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

    /// Add a new keyframe
    /**
     * \param[in] frame a new keyframe
     */
    void addKeyfame(vslam_datastructure::Frame::SharedPtr keyframe) override;

    /// Remove a keyframe
    /**
     * \param[in] frame the keyframe to be removed
     */
    void removeKeyframe(vslam_datastructure::Frame::SharedPtr keyframe) override;

    /// Get a keyframe using the id. Return a nullptr if the keyframe cannot be found
    vslam_datastructure::Frame::SharedPtr getKeyframe(const long unsigned int id) const override;

    /// Add a loop constraint and run pose-graph optimization
    /**
     * \param[in] kf_id_1 the first keyframe id
     * \param[in] kf_id_2 the second keyframe id
     * \param[in] T_1_2 relative transformation from the first to the second keyframe
     * \param[in] sim3_scale the similarity transform scale
     */
    void addLoopConstraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2,
                           const double sim3_scale) override;

    /// Convert all the keyframes to frame msgs to refresh the visualizer
    std::vector<vslam_msgs::msg::Frame> getAllKeyframeMsgs() const override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_backend_plugins::IndirectOptimizer"; }

  private:
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
    void localBALoop();

    /// Get the latest `num_core_kfs_` core keyframes and their map points
    /**
     * \return core keyframes and map points
     */
    std::pair<CoreKfsSet, CoreMpsSet> getCoreKeyframesMappoints();

    /// Cleanup the outlier map points and keyframes
    void cleanUpStaleKeyframesMappoints();

    /// Flag indicating stale keyframes and map points are being cleaned up
    std::atomic_bool cleaning_stale_keyframes_mappoints_{false};

    /// Number of core keyframes used in local BA
    size_t num_core_kfs_{5};

    /// A flag to indicate if the pose-graph optimization is running
    std::atomic_bool loop_optimization_running_{false};

    /// A boolean to stop the thread
    std::atomic_bool exit_thread_{false};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
