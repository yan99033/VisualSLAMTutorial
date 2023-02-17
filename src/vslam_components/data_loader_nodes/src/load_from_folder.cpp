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

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <filesystem>
#include <fmt/format.h>
#undef NDEBUG
#include <cassert>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
  std::vector<std::string> load_files(const std::string& folder, std::string ext = ".png") {
    std::vector<std::string> files;

    // Check if the path points to an empty file or directory
    // asssert
    // if (!std::filesystem::exists(folder) or std::filesystem::is_empty(folder)) {
    // throw ImageLoaderError(
    //     fmt::format("The folder ({}) is either invalid or empty.\n", folder));
    // }

    for (const auto& f : std::filesystem::directory_iterator(folder)) {
      if (f.path().extension() == ext) {
          files.push_back(f.path());
      }
    }

    assert(!files.empty());

    std::sort(files.begin(), files.end());

    return files;
  }

  std::string mat_type2encoding(int mat_type) {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
  }
}

namespace vslam_components {

namespace data_loader_nodes {

LoadFromFolder::LoadFromFolder(const rclcpp::NodeOptions & options)
: Node("load_from_folder", options), count_(0), files_{load_files("/Users/sy/kitti/00/image_2")}
{
  // Create a publisher of "vslam_msgs/Frame" messages on the "chatter" topic.
  pub_ = create_publisher<vslam_msgs::msg::Frame>("camera_node", 10);

  // Use a timer to schedule periodic message publishing.
  auto frame_rate_hz = declare_parameter("frame_rate_hz", 10);
  assert(frame_rate_hz > 0);
  timer_ = create_wall_timer(std::chrono::duration<double>(1. / frame_rate_hz), std::bind(&LoadFromFolder::on_timer, this));
  
  RCLCPP_INFO(this->get_logger(), "Going to publish the frames at '%lu' hz \n", frame_rate_hz);
  RCLCPP_INFO(this->get_logger(), "Loaded '%lu' files\n", files_.size());
}

void LoadFromFolder::on_timer()
{
  // Load image
  cv::Mat image = cv::imread(files_.at(count_ % files_.size()),  cv::IMREAD_COLOR);
  sensor_msgs::msg::Image im_msg;
  im_msg.height = image.rows;
  im_msg.width = image.cols;
  im_msg.encoding = mat_type2encoding(image.type());
  im_msg.is_bigendian = false;
  im_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  im_msg.data.assign(image.datastart, image.dataend);  

  // Create a frame
  auto msg = std::make_unique<vslam_msgs::msg::Frame>();
  msg->id = ++count_;
  RCLCPP_INFO(this->get_logger(), fmt::format("Publishing frame: {} / {}", msg->id, files_.size()).c_str());
  msg->image = im_msg;

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