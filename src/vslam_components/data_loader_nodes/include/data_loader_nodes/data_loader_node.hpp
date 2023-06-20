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

#ifndef DATA_LOADER_NODES__DATA_LOADER_NODE_HPP_
#define DATA_LOADER_NODES__DATA_LOADER_NODE_HPP_

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>

#include "camera_plugins_base/monocular_camera.hpp"
#include "data_loader_nodes/camera_info.hpp"
#include "vslam_msgs/msg/frame.hpp"
#include "vslam_utils/undistorter.hpp"

namespace vslam_components {

  namespace data_loader_nodes {
    namespace abstract {
      class DataLoaderNode {
      protected:
        /// Periodically call this function to publish data
        virtual void onTimer() = 0;
      };
    }  // namespace abstract

    class DataLoaderNode : public virtual abstract::DataLoaderNode, public rclcpp::Node {
    public:
      explicit DataLoaderNode(const rclcpp::NodeOptions& options);

      ~DataLoaderNode();

    protected:
      void onTimer() override;

    private:
      /// Frame count
      int count_{0};

      /// Frame publisher
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;

      /// Wall timer to publish frame data
      rclcpp::TimerBase::SharedPtr timer_;

      /// Camera matrix (based on undistorted images)
      sensor_msgs::msg::CameraInfo cam_info_msg_;

      /// Camera plugin loader
      pluginlib::ClassLoader<camera_plugins::base::MonocularCamera> camera_loader_{
          "camera_plugins_base", "camera_plugins::base::MonocularCamera"};

      /// Camera plugin
      std::shared_ptr<camera_plugins::base::MonocularCamera> camera_;
    };
  }  // namespace data_loader_nodes
}  // namespace vslam_components

#endif  // DATA_LOADER_NODES__DATA_LOADER_NODE_HPP_