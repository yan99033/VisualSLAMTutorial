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

#ifndef MONOCULAR_CAMERA_PLUGINS__CALICAM_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS__CALICAM_PLUGIN_HPP_

#include "monocular_camera_plugins/monocular_camera.hpp"

namespace monocular_camera_plugins {
  // For more information about the camera, see here
  // https://github.com/astar-ai/calicam
  /// \note I haven't done extensive tests with the camera yet
  class CaliCam : public MonocularCamera {
  public:
    /// Calicam initializer
    void initialize(const std::string& params) override;
  };
}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS__CALICAM_PLUGIN_HPP_