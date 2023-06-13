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

#ifndef VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_
#define VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"

namespace vslam_feature_matcher_plugins {

  class OpenCVFeatureMatcher : public virtual vslam_feature_matcher::base::FeatureMatcher {
  public:
    /// Destructor
    ~OpenCVFeatureMatcher() { std::cerr << "Terminated OpenCVFeatureMatcher" << std::endl; }

    /// Feature matcher initializer
    void initialize(const vslam_datastructure::Point::Type point_type) override;

    /// Match features between two sets of points
    /**
     * \param[in] points1 a vector of points from frame 1
     * \param[in] points2 a vector of points from frame 2
     * \return matches between the two vector of points
     */
    vslam_datastructure::Matches matchFeatures(const vslam_datastructure::Points& points1,
                                               const vslam_datastructure::Points& points2) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_feature_matcher_plugins::OpenCVFeatureMatcher"; }

  private:
    cv::Ptr<cv::DescriptorMatcher> feature_matcher_;
  };
}  // namespace vslam_feature_matcher_plugins

#endif  // VSLAM_FEATURE_MATCHER_PLUGINS__ORB_HPP_
