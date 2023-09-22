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

#ifndef VSLAM_PLUGINS_BASE__MAPPER_HPP_
#define VSLAM_PLUGINS_BASE__MAPPER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_mapper {
  namespace base {
    class Mapper : public virtual vslam_plugin::base::Plugin {
    public:
      /// Mapper initializer
      virtual void initialize() = 0;

      /// Create new map points based on the point correspondences and the pose constraints
      /**
       * \param[in] matched_points Point correspndences
       * \param[in] T_1_w camera pose of the first frane
       * \param[in] T_2_1 camera pose from the first to second frame
       * \param[in] K camera matrix
       */
      virtual vslam_datastructure::MapPoints map(const vslam_datastructure::MatchedPoints& matched_points,
                                                 const cv::Mat& T_1_w, const cv::Mat& T_2_1, const cv::Mat& K)
          = 0;

      /// Destructor
      virtual ~Mapper() {}

    protected:
      /// Constructor
      Mapper() {}
    };
  }  // namespace base
}  // namespace vslam_mapper

#endif  // VSLAM_PLUGINS_BASE__MAPPER_HPP_