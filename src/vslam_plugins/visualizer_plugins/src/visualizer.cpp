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

#include "vslam_visualizer_plugins/visualizer.hpp"

namespace vslam_visualizer_plugins {

  const Eigen::Matrix3d Visualizer::cam_axes_transform_
      = (Eigen::Matrix3d() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0).finished();

  const Mat4x16 Visualizer::cam_marker_vertices_
      = (Mat4x16() << Visualizer::p0[0], Visualizer::p1[0], Visualizer::p0[0], Visualizer::p2[0], Visualizer::p0[0],
         Visualizer::p3[0], Visualizer::p0[0], Visualizer::p4[0], Visualizer::p4[0], Visualizer::p3[0],
         Visualizer::p3[0], Visualizer::p2[0], Visualizer::p2[0], Visualizer::p1[0], Visualizer::p1[0],
         Visualizer::p4[0],  //
         Visualizer::p0[1], Visualizer::p1[1], Visualizer::p0[1], Visualizer::p2[1], Visualizer::p0[1],
         Visualizer::p3[1], Visualizer::p0[1], Visualizer::p4[1], Visualizer::p4[1], Visualizer::p3[1],
         Visualizer::p3[1], Visualizer::p2[1], Visualizer::p2[1], Visualizer::p1[1], Visualizer::p1[1],
         Visualizer::p4[1],  //
         Visualizer::p0[2], Visualizer::p1[2], Visualizer::p0[2], Visualizer::p2[2], Visualizer::p0[2],
         Visualizer::p3[2], Visualizer::p0[2], Visualizer::p4[2], Visualizer::p4[2], Visualizer::p3[2],
         Visualizer::p3[2], Visualizer::p2[2], Visualizer::p2[2], Visualizer::p1[2], Visualizer::p1[2],
         Visualizer::p4[2],  //
         Visualizer::p0[3], Visualizer::p1[3], Visualizer::p0[3], Visualizer::p2[3], Visualizer::p0[3],
         Visualizer::p3[3], Visualizer::p0[3], Visualizer::p4[3], Visualizer::p4[3], Visualizer::p3[3],
         Visualizer::p3[3], Visualizer::p2[3], Visualizer::p2[3], Visualizer::p1[3], Visualizer::p1[3],
         Visualizer::p4[3])
            .finished();

  Mat3x16 Visualizer::getCameraVertices(const double scale, const Eigen::Isometry3d& T_w_c) {
    Mat4x16 cam_marker_vertices = cam_marker_vertices_;
    cam_marker_vertices.topRows(3) = scale * cam_marker_vertices.topRows(3);
    cam_marker_vertices = T_w_c.matrix() * cam_marker_vertices;

    return cam_marker_vertices.topRows(3);
  }

}  // namespace vslam_visualizer_plugins