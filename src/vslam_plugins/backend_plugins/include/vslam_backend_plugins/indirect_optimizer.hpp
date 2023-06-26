/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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
#include "vslam_datastructure/map.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {
  class IndirectOptimizer : public vslam_backend_plugins::Optimizer {
  public:
    ~IndirectOptimizer();

    /// Indirect optimizer initializer
    /**
     * \param[in] map the map object comprising the keyframes
     */
    void initialize(vslam_datastructure::Map* map) override;

    /// Run local BA
    /**
     * Non-blocking call that runs local bundle adjustment on the core keyframes and map points, which can be obtained
     * from the map
     */
    void runLocalBA() override;

    /// Add a loop constraint and run pose-graph optimization
    /**
     * \param[in] kf_id_1 the first keyframe id
     * \param[in] kf_id_2 the second keyframe id
     * \param[in] T_1_2 relative transformation from the first to the second keyframe
     * \param[in] sim3_scale the similarity transform scale
     */
    void addLoopConstraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2, const cv::Mat& T_1_2,
                           const double sim3_scale) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_backend_plugins::IndirectOptimizer"; }

  private:
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

    /// Cleanup the outlier map points and keyframes
    /**
     * \param[in] min_kf_id keyframe ids less than this won't be processed
     * \param[in] max_kf_id keyframe ids greater than this won't be processed
     */
    void cleanUpStaleKeyframesMappoints(const long unsigned int min_kf_id = 0,
                                        const long unsigned int max_kf_id = ULONG_MAX);

    /// Flag indicating stale keyframes and map points are being cleaned up
    std::atomic_bool cleaning_stale_keyframes_mappoints_{false};

    /// Number of core keyframes used in local BA
    size_t num_core_kfs_{5};

    /// Maximum relative rotation between two consecutive keyframes to detect bad keyframes
    /// Normally the relative rotation is large if tracking fails. Defaults to 60 degrees
    static constexpr const double max_outlier_rel_rotation_rad_{1.0472};

    /// Scale of the maximum relative translation to detect bad keyframes
    /// The current relative translation must not be larger than the scale times the previous relative translation.
    /// Normally we get a big pose jump if tracking fails
    static constexpr const double outlier_rel_translation_scale_{10.0};

    /// A flag to indicate if the pose-graph optimization is running
    std::atomic_bool loop_optimization_running_{false};

    /// A boolean to stop the thread
    std::atomic_bool exit_thread_{false};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__INDIRECT_HPP_
