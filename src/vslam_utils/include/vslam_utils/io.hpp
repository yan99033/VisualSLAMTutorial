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

#ifndef VSLAM_UTILS__IO_HPP_
#define VSLAM_UTILS__IO_HPP_

#include <string>

namespace vslam_utils {

  namespace io {
    /// Get the filenames in a folder
    /**
     * \param folder[in] path to a folder containing the images with the image extention of .png or .jpg
     * \return a vector containing the image filenames in the folder
     */
    std::vector<std::string> loadFromFolder(const std::string& folder);

  }  // namespace io

}  // namespace vslam_utils

#endif  // VSLAM_UTILS__IO_HPP_