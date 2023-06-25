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

#ifndef VSLAM_NODES__VSLAM_NODE_BASE_HPP_
#define VSLAM_NODES__VSLAM_NODE_BASE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "vslam_datastructure/frame.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    namespace abstract {
      class VSlamNode {
      public:
        /// Destructor
        virtual ~VSlamNode() {}

        /// Process the frame depending on the camera tracking states
        /**
         * \param current_frame[in,out] current frame received from the camera
         */
        virtual void processFrame(vslam_datastructure::Frame::SharedPtr current_frame) = 0;

      protected:
        /// Process the frame in the 'init' state
        /**
         * \param current_frame[in,out] current frame received from the camera
         * \return a boolean indicating if initialization is successful
         */
        virtual bool processFrameInit(vslam_datastructure::Frame::SharedPtr current_frame) = 0;

        /// Process the frame in the 'attempt_init' state
        /**
         * \param current_frame[in,out] current frame received from the camera
         * \return a boolean indicating if the attempt to initialize is successful
         */
        virtual bool processFrameAttemptInit(vslam_datastructure::Frame::SharedPtr current_frame) = 0;

        /// Process the frame in the 'tracking' state
        /**
         * \param current_frame[in,out] current frame received from the camera
         * \return a boolean indicating if tracking is successful
         */
        virtual bool processFrameTracking(vslam_datastructure::Frame::SharedPtr current_frame) = 0;

        /// Process the frame in the 'relocalization' state
        /**
         * \param current_frame[in,out] current frame received from the camera
         * \return a boolean indicating if relocalization is successful
         */
        virtual bool processFrameRelocalization(vslam_datastructure::Frame::SharedPtr current_frame) = 0;

        VSlamNode() {}
      };
    }  // namespace abstract

    class VSlamNode : public virtual abstract::VSlamNode {
    public:
      void processFrame(vslam_datastructure::Frame::SharedPtr current_frame) override;

    private:
      /// Camera tracking states
      /**
       * init: No keyframe is available for tracking. Tentatively set the current frame as a keyframe
       * attempt_init: Attempt to initialize the keyframe by calculating the relative transformation and map points
       *               between the keyframe and the current frame. Remove the keyframe and reset the state to init if
       *               initialization is not successful.
       * tracking: A keyframe is available for tracking. Track the current frame and determine if a new keyframe is
       *           needed. Triangulate new map points if a new keyframe is created
       * relocalization: Tracking is bad. Attempt to gain tracking using the current keyframe or the keyframe similar
       *                 to one of the keyframes found by place recognition
       */
      enum class State : uint8_t { init = 0, attempt_init = 1, tracking = 2, relocalization = 3 };

      /// State of the node
      State state_{State::init};
    };
  }  // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__VSLAM_NODE_BASE_HPP_