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

#ifndef VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
#define VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_

#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/mapper.hpp"

namespace vslam_mapper_plugins {

  class OpenCVMapper : public virtual vslam_mapper::base::Mapper {
  public:
    /// Destructor
    ~OpenCVMapper() { std::cerr << "Terminated OpenCVMapper" << std::endl; }

    /// Mapper initializer
    void initialize() override {}

    /// Create new map points based on the point correspondences and the pose constraints
    /**
     * \param[in] matched_points Point correspndences
     * \param[in] T_1_w camera pose of the first frane
     * \param[in] T_2_1 camera pose from the first to second frame
     * \param[in] K camera matrix
     */
    virtual vslam_datastructure::MapPoints map(const vslam_datastructure::MatchedPoints& matched_points,
                                               const cv::Mat& T_1_w, const cv::Mat& T_2_1, const cv::Mat& K) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_mapper_plugins::OpenCVMapper"; }

  private:
    /// Threshold to remove map points that have a large reprojection error
    static constexpr const double proj_err_thresh_{5.0};
  };
}  // namespace vslam_mapper_plugins

#endif  // VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
