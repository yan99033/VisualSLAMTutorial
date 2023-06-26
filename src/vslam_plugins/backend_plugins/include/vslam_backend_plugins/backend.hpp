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

#ifndef VSLAM_BACKEND_PLUGINS__BACKEND_HPP_
#define VSLAM_BACKEND_PLUGINS__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/map.hpp"
#include "vslam_plugins_base/backend.hpp"

namespace vslam_backend_plugins {
  namespace abstract {
    class Optimizer : public virtual vslam_backend::base::Backend {
    protected:
      /// Implementation of bundle adjustment
      /**
       * \param[in,out] core_keyframes core keyframes to optimize. The current keyframe and the other keyframes will be
       *    fixed during optimization
       * \param[in,out] core_mappoints core map points to optimize.
       */
      virtual void runBundleAdjustmentImpl(vslam_datastructure::CoreKfsSet& core_keyframes,
                                           vslam_datastructure::CoreMpsSet& core_mappoints)
          = 0;

      /// Implementation of pose graph optimization
      /**
       * \param[in] kf_id_1 the first keyframe id of the loop constraint
       * \param[in] kf_id_2 the second keyframe id of the loop constraint
       * \param[in] T_1_2 the relative pose constraint
       * \param[in] sim3_scale the Sim(3) scale of the loop constraint
       * \param[in,out] keyframes the keyframes in the map
       */
      virtual void runPoseGraphOptimizationImpl(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                                const cv::Mat& T_1_2, const double sim3_scale,
                                                std::list<vslam_datastructure::Frame::SharedPtr>& keyframes)
          = 0;
    };
  }  // namespace abstract

  class Optimizer : public virtual abstract::Optimizer {
  protected:
    ~Optimizer();

    /// Implementation of bundle adjustment
    /**
     * \param[in,out] core_keyframes core keyframes to optimize. The current keyframe and the other keyframes will be
     *    fixed during optimization
     * \param[in,out] core_mappoints core map points to optimize.
     */
    void runBundleAdjustmentImpl(vslam_datastructure::CoreKfsSet& core_keyframes,
                                 vslam_datastructure::CoreMpsSet& core_mappoints) override;

    /// Implementation of pose graph optimization
    /**
     * \param[in] kf_id_1 the first keyframe id of the loop constraint
     * \param[in] kf_id_2 the second keyframe id of the loop constraint
     * \param[in] T_1_2 the relative pose constraint
     * \param[in] sim3_scale the Sim(3) scale of the loop constraint
     * \param[in,out] keyframes the keyframes in the map
     */
    void runPoseGraphOptimizationImpl(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                      const cv::Mat& T_1_2, const double sim3_scale,
                                      std::list<vslam_datastructure::Frame::SharedPtr>& keyframes) override;

    /// (Non-owning) map comprising the keyframes
    vslam_datastructure::Map* map_;

    /// The window size of the error
    static constexpr const double huber_kernel_delta_{2.4477};

    /// A squared error above this is considered as outlier in the data
    static constexpr const double huber_kernel_delta_sq_{5.991};

    /// Bundle adjustment optimization iterations
    static constexpr const int ba_iterations_{15};

    /// Pose-graph optimization iterations
    static constexpr const int pgo_iterations_{15};
  };
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__BACKEND_HPP_