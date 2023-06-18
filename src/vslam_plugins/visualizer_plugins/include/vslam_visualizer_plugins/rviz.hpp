/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_
#define VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "vslam_msgs/msg/frame.hpp"
#include "vslam_visualizer_plugins/visualizer.hpp"

namespace vslam_visualizer_plugins {

  class RViz : public Visualizer {
  public:
    ~RViz();

    /// RViz visualizer initializer
    void initialize() override;

    /// Add a live camera pose to the visualizer (which may not be a keyframe)
    /**
     * \param[in] frame_msg Frame message
     */
    void addLiveFrame(const vslam_msgs::msg::Frame& frame_msg) override;

    /// Add a keyframe camera pose and its map points to the visualizer
    /**
     * \param[in] frame_msg Frame message
     */
    void addKeyframe(const vslam_msgs::msg::Frame& frame_msg) override;

    /// Remove a keyframe camera pose and its map points to the visualizer
    /**
     * \warning NOT implemented
     * \param[in] frame_msg Frame message
     */
    void removeKeyframe(const vslam_msgs::msg::Frame& frame_msg) override;

    /// Replace all keyframes and their map points in the visualizer
    /**
     * \param[in] frame_msgs a vector of type Frame message
     */
    void replaceAllKeyframes(const FrameVec& frame_msgs) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_visualizer_plugins::RViz"; }

  private:
    /// Visualizer node
    rclcpp::Node::SharedPtr node_{nullptr};

    /// Image publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    /// Live frame publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr live_frame_publisher_;

    /// Keyframe publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keyframe_publisher_;

    /// Map point publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mappoint_publisher_;

    /// Camera pose publisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /// The frame id of the global map frame
    std::string map_frame_id_{"map"};

    /// The frame id of the camera frame
    std::string cam_frame_id_{"cam"};

    /// Live camera marker scale
    double live_cam_marker_scale_{10.0};

    /// Live camera marker namespace
    std::string live_cam_marker_ns_{"live"};

    /// Live camera marker colour
    std::array<double, 3> live_cam_marker_rgb_{1.0, 0.0, 0.0};

    /// Scale of keyframe and map point markers
    double marker_scale_{0.25};

    /// Keyframe camera marker namespace
    std::string keyframe_cam_marker_ns_{"keyframe_poses"};

    /// Keyframe camera marker colour
    std::array<double, 3> keyframe_cam_marker_rgb_{0.0, 1.0, 0.0};

    /// Map point marker namespace
    std::string keyframe_mps_marker_ns_{"keyframe_mps"};

    /// Map point marker colour
    std::array<double, 3> mappoint_marker_rgb_{0.0, 0.0, 0.0};

    /// Line thickness of the markers
    double line_thickness_{1.0};
  };
}  // namespace vslam_visualizer_plugins

#endif  // VSLAM_VISUALIZER_PLUGINS__RVIZ_HPP_
