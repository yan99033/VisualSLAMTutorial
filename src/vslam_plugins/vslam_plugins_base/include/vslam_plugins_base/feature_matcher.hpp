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

#ifndef VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_
#define VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_feature_matcher {
  namespace base {
    class FeatureMatcher : public virtual vslam_plugin::base::Plugin {
    public:
      /// Feature matcher initializer
      virtual void initialize(const vslam_datastructure::Point::Type point_type) = 0;

      /// Match features between two sets of points
      /**
       * \param[in] points1 a vector of points from frame 1
       * \param[in] points2 a vector of points from frame 2
       * \return matches between the two vector of points
       */
      virtual vslam_datastructure::MatchedPoints matchFeatures(const vslam_datastructure::Points& points1,
                                                               const vslam_datastructure::Points& points2)
          = 0;

      /// Destructor
      virtual ~FeatureMatcher() {}

    protected:
      /// Constructor
      FeatureMatcher() {}
    };
  }  // namespace base
}  // namespace vslam_feature_matcher

#endif  // VSLAM_PLUGINS_BASE__FEATURE_MATCHER_HPP_