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

#include "vslam_datastructure/typedefs.hpp"

#include "vslam_datastructure/frame.hpp"
#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {
  bool FrameCmp::operator()(Frame::SharedPtr const lhs, Frame::SharedPtr const rhs) const {
    if (lhs && rhs) {
      return lhs->id() < rhs->id();
    } else {
      return false;
    }
  }

}  // namespace vslam_datastructure