// TODO: licence

#include "data_loader_nodes/data_loader_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace {
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
    DataLoaderNode::DataLoaderNode(const rclcpp::NodeOptions &options)
        : Node("load_from_folder_node", options),
          frame_pub_{create_publisher<vslam_msgs::msg::Frame>("out_frame", 10)} {
      // Use a timer to schedule periodic message publishing.
      const auto frame_rate_hz = declare_parameter("frame_rate_hz", 10);
      assert(frame_rate_hz > 0);
      timer_ = create_wall_timer(std::chrono::duration<double>(1. / frame_rate_hz),
                                 std::bind(&DataLoaderNode::on_timer, this));

      RCLCPP_INFO(this->get_logger(), "Going to publish the frames at %llu hz \n", frame_rate_hz);

      const auto plugin_name = declare_parameter("plugin_name", "UNDEFINED");
      const auto cam_params_file = declare_parameter("params_file", "");
      camera_ = camera_loader_.createSharedInstance(plugin_name);
      camera_->initialize(cam_params_file);

      cam_info_msg_ = toCameraInfoMsg(camera_->K());
    }

    void DataLoaderNode::on_timer() {
      const auto image = camera_->grab_image();

      if (image.empty()) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Received an empty image. Skip publishing \n");
        return;
      }

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
      RCLCPP_INFO(this->get_logger(), "Publishing frame: %u", msg->id);
      msg->image = im_msg;
      msg->cam_info = cam_info_msg_;
      msg->header.stamp = now();

      // Put the message into a queue to be processed by the middleware.
      frame_pub_->publish(std::move(msg));
    }
  }  // namespace data_loader_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::data_loader_nodes::DataLoaderNode)