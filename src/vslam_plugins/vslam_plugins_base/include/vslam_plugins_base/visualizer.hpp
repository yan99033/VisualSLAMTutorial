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

#ifndef VSLAM_PLUGINS_BASE__VISUALIZER_HPP_
#define VSLAM_PLUGINS_BASE__VISUALIZER_HPP_

#include "vslam_msgs/msg/frame.hpp"
#include "vslam_plugins_base/base.hpp"

namespace vslam_visualizer {
  namespace base {
    class Visualizer : public virtual vslam_plugin::base::Plugin {
    public:
      using FrameVec = std::vector<vslam_msgs::msg::Frame>;

      /// Visualizer initializer
      virtual void initialize() = 0;

      /// Add a live camera pose to the visualizer (which may not be a keyframe)
      /**
       * \param[in] frame_msg Frame message
       */
      virtual void addLiveFrame(const vslam_msgs::msg::Frame& frame_msg) = 0;

      /// Add a keyframe camera pose and its map points to the visualizer
      /**
       * \param[in] frame_msg Frame message
       */
      virtual void addKeyframe(const vslam_msgs::msg::Frame& frame_msg) = 0;

      /// Remove a keyframe camera pose and its map points to the visualizer
      /**
       * \param[in] frame_msg Frame message
       */
      virtual void removeKeyframe(const vslam_msgs::msg::Frame& frame_msg) = 0;

      /// Replace all keyframes and their map points in the visualizer
      /**
       * \param[in] frame_msgs a vector of type Frame message
       */
      virtual void replaceAllKeyframes(const FrameVec& frame_msgs) = 0;

      /// Destructor
      virtual ~Visualizer() {}

    protected:
      /// Constructor
      Visualizer() {}
    };
  }  // namespace base
}  // namespace vslam_visualizer

#endif  // VSLAM_PLUGINS_BASE__VISUALIZER_HPP_