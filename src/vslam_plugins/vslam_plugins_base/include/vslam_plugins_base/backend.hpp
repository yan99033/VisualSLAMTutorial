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

#ifndef VSLAM_PLUGINS_BASE__BACKEND_HPP_
#define VSLAM_PLUGINS_BASE__BACKEND_HPP_

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/map.hpp"
#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_backend {
  namespace base {
    class Backend : public virtual vslam_plugin::base::Plugin {
    public:
      /// Back-end initializer
      /**
       * \param[in] map the map object comprising the keyframes
       */
      virtual void initialize(vslam_datastructure::Map* map) = 0;

      /// Run local bundle adjustment
      virtual void runLocalBA() = 0;

      /// Add a loop constraint and run pose-graph optimization
      /**
       * \param[in] kf_id_1 the first keyframe id
       * \param[in] kf_id_2 the second keyframe id
       * \param[in] T_1_2 relative transformation from the first to the second keyframe
       * \param[in] sim3_scale the similarity transform scale
       */
      virtual void addLoopConstraint(const long unsigned int kf_id_1, const long unsigned int kf_id_2,
                                     const cv::Mat& T_1_2, const double sim3_scale)
          = 0;

      /// Destructor
      virtual ~Backend() {}

    protected:
      /// Constructor
      Backend() {}
    };
  }  // namespace base
}  // namespace vslam_backend

#endif  // VSLAM_PLUGINS_BASE__BACKEND_HPP_