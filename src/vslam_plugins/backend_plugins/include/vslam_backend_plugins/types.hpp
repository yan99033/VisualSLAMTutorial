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

#ifndef VSLAM_BACKEND_PLUGINS__TYPES_HPP_
#define VSLAM_BACKEND_PLUGINS__TYPES_HPP_

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_backend_plugins {
  namespace types {
    /// Datastructure for keeping the correspondences between the vertices and edges in the optimizer and the frames,
    /// map points and projections in the visual SLAM map
    struct SparseBAResults {
      /// Map from the g2o map point vertices to the map points
      std::unordered_map<vslam_datastructure::MapPoint::SharedPtr, g2o::VertexPointXYZ*> mappoint_vertices;

      /// Map from the g2o frame vertices to the keyframes
      std::unordered_map<vslam_datastructure::Frame::SharedPtr, g2o::VertexSE3Expmap*> keyframe_vertices;

      /// Map from the g2o edges to the map point projections
      std::unordered_map<g2o::EdgeSE3ProjectXYZ*, vslam_datastructure::PointMappointPair> edges;
    };

    using poseGraphOptimizationResults
        = std::unordered_map<vslam_datastructure::Frame::SharedPtr, g2o::VertexSim3Expmap*>;
  }  // namespace types
}  // namespace vslam_backend_plugins

#endif  // VSLAM_BACKEND_PLUGINS__TYPES_HPP_