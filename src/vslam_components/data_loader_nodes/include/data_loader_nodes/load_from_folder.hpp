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

#ifndef VSLAM_COMPONENTS__LOAD_FROM_FOLDER_HPP_
#define VSLAM_COMPONENTS__LOAD_FROM_FOLDER_HPP_

#include "data_loader_nodes/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {

namespace data_loader_nodes {

class LoadFromFolder : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LoadFromFolder(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace data_loader_nodes

}  // namespace composition

#endif  // VSLAM_COMPONENTS__LOAD_FROM_FOLDER_HPP_