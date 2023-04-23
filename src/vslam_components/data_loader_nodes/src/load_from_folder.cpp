// TODO: licence

#include "data_loader_nodes/load_from_folder.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <utility>
#undef NDEBUG
#include <cassert>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
  std::vector<std::string> load_files(const std::string &folder, std::string ext = ".png") {
    std::vector<std::string> files;

    assert(std::filesystem::exists(folder) || !std::filesystem::is_empty(folder));

    for (const auto &f : std::filesystem::directory_iterator(folder)) {
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
}  // namespace

namespace vslam_components {

  namespace data_loader_nodes {

    LoadFromFolder::LoadFromFolder(const rclcpp::NodeOptions &options)
        : Node("load_from_folder_node", options), count_(0), files_{load_files(declare_parameter("image_folder", ""))} {
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);

      // Use a timer to schedule periodic message publishing.
      auto frame_rate_hz = declare_parameter("frame_rate_hz", 10);
      assert(frame_rate_hz > 0);
      timer_ = create_wall_timer(std::chrono::duration<double>(1. / frame_rate_hz),
                                 std::bind(&LoadFromFolder::on_timer, this));

      RCLCPP_INFO(this->get_logger(), "Going to publish the frames at %llu hz \n", frame_rate_hz);
      RCLCPP_INFO(this->get_logger(), "Loaded '%lu' files\n", files_.size());
    }

    void LoadFromFolder::on_timer() {
      // Load image
      cv::Mat image = cv::imread(files_.at(count_ % files_.size()), cv::IMREAD_COLOR);
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
      RCLCPP_INFO(this->get_logger(), "Publishing frame: %u / %lu", msg->id, files_.size());
      msg->image = im_msg;
      msg->header.stamp = now();

      // Put the message into a queue to be processed by the middleware.
      // This call is non-blocking.
      frame_pub_->publish(std::move(msg));
    }
  }  // namespace data_loader_nodes

}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::data_loader_nodes::LoadFromFolder)