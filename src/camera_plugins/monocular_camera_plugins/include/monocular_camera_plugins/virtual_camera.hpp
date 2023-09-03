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

#ifndef MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_
#define MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_

#include "camera_plugins_base/monocular_camera.hpp"
#include "monocular_camera_plugins/monocular_camera.hpp"

namespace monocular_camera_plugins {
  class VirtualCamera : public MonocularCamera {
  public:
    /// Virtual camera initializer
    void initialize(const std::string& params_file) override;

    /// Grab the next image from the folder
    /**
     * \return An undistorted image
     */
    cv::Mat grabImage() override;

    /// Get the camera matrix
    /**
     * \return Camera matrix
     */
    cv::Mat K() override { return K_.clone(); }

  private:
    /// The current index in the files_ vector
    size_t i_{0};

    /// filenames in a sequential order
    std::vector<std::string> files_;
  };

}  // namespace monocular_camera_plugins

#endif  // MONOCULAR_CAMERA_PLUGINS_VIRTUAL_CAMERA_PLUGIN_HPP_