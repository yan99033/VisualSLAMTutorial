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

#include "vslam_nodes/vslam_node_base.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    void VSlamNode::processFrame(vslam_datastructure::Frame::SharedPtr current_frame) {
      if (state_ == State::init) {
        if (processFrameInit(current_frame)) {
          state_ = State::attempt_init;
        }
      } else if (state_ == State::attempt_init) {
        if (processFrameAttemptInit(current_frame)) {
          state_ = State::tracking;
        } else {
          state_ = State::init;
        }
      } else if (state_ == State::tracking) {
        if (!processFrameTracking(current_frame)) {
          state_ = State::relocalization;
        }
      } else if (state_ == State::relocalization) {
        if (processFrameRelocalization(current_frame)) {
          state_ = State::tracking;
        }
      }
    }

  }  // namespace vslam_nodes
}  // namespace vslam_components