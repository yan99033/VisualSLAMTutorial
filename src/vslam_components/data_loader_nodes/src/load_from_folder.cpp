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

  sensor_msgs::msg::CameraInfo toCameraInfoMsg(const cv::Mat &K) {
    assert(K.rows == 3 && K.cols == 3);
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.k = {K.at<double>(0, 0), 0, K.at<double>(0, 2), 0, K.at<double>(1, 1), K.at<double>(1, 2), 0, 0, 1};

    return cam_info_msg;
  }

}  // namespace

namespace vslam_components {

  namespace data_loader_nodes {

    LoadFromFolder::LoadFromFolder(const rclcpp::NodeOptions &options)
        : Node("load_from_folder_node", options), count_(0), files_{load_files(declare_parameter("image_folder", ""))} {
      const auto cam_info = load_camera_info();

      // Undistorter
      undistorter_ = std::make_unique<vslam_utils::Undistorter>(cam_info.K, cam_info.image_width, cam_info.image_height,
                                                                cam_info.dist_coeffs);
      cam_info_msg_ = toCameraInfoMsg(undistorter_->get_K());

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
      // Stop at the end of the sequence
      if (count_ >= files_.size() - 1) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Reached the end of the sequence. Stop publishing new frames \n");
        return;
      }

      // Load image
      cv::Mat image = cv::imread(files_.at(count_ % files_.size()), cv::IMREAD_COLOR);
      image = undistorter_->undistort_image(image);
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
      msg->cam_info = cam_info_msg_;
      msg->header.stamp = now();

      // Put the message into a queue to be processed by the middleware.
      frame_pub_->publish(std::move(msg));
    }

    CameraInfo LoadFromFolder::load_camera_info() {
      double fx = declare_parameter("camera_info.camera_matrix.fx", -1.0);
      double fy = declare_parameter("camera_info.camera_matrix.fy", -1.0);
      double cx = declare_parameter("camera_info.camera_matrix.cx", -1.0);
      double cy = declare_parameter("camera_info.camera_matrix.cy", -1.0);
      if (fx <= 0.0 || fy <= 0.0 || cx <= 0.0 || cy <= 0.0) {
        throw std::runtime_error("Invalid camera info. Ensure fx, fy, cx, cy are loaded and greater than zero.");
      }

      cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

      std::vector<double> dist_coeffs_array
          = declare_parameter("camera_info.dist_coeffs", std::vector<double>{0.0, 0.0, 0.0, 0.0});
      if (dist_coeffs_array.size() != 4 && dist_coeffs_array.size() != 5) {
        throw std::runtime_error("Distortion coefficients array has to be of size 4 or 5. Got "
                                 + std::to_string(dist_coeffs_array.size()));
      }
      cv::Mat dist_coeffs(dist_coeffs_array.size(), 1, CV_64F, dist_coeffs_array.data());
      dist_coeffs = dist_coeffs.clone();

      int image_height = declare_parameter("camera_info.image_height", 0);
      int image_width = declare_parameter("camera_info.image_width", 0);
      if (image_height <= 0 || image_width <= 0) {
        throw std::runtime_error("Invalid image size. Width and height have to be greater than zero.");
      }

      CameraInfo cam_info{K, dist_coeffs, image_height, image_width};

      return cam_info;
    }
  }  // namespace data_loader_nodes

}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::data_loader_nodes::LoadFromFolder)