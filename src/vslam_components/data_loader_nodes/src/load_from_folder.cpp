// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "data_loader_nodes/load_from_folder.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace vslam_components {

namespace data_loader_nodes {

LoadFromFolder::LoadFromFolder(const rclcpp::NodeOptions & options)
: Node("load_from_folder", options), count_(0)
{
  // Create a publisher of "vslam_msgs/Frame" messages on the "chatter" topic.
  pub_ = create_publisher<vslam_msgs::msg::Frame>("camera_node", 10);

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(1s, std::bind(&LoadFromFolder::on_timer, this));
}

void LoadFromFolder::on_timer()
{
  auto msg = std::make_unique<vslam_msgs::msg::Frame>();
  msg->id = ++count_;
  RCLCPP_INFO(this->get_logger(), "Publishing frame: '%d'", msg->id);
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  pub_->publish(std::move(msg));
}

}  // namespace data_loader_nodes

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::data_loader_nodes::LoadFromFolder)