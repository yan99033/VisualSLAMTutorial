#include "vslam_nodes/indirect_vslam_node.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include "vslam_msgs/msg/vector2d.hpp"
#include "vslam_msgs/msg/vector3d.hpp"

using std::placeholders::_1;

namespace {
  int encoding2mat_type(const std::string &encoding) {
    if (encoding == "mono8") {
      return CV_8UC1;
    } else if (encoding == "bgr8") {
      return CV_8UC3;
    } else if (encoding == "mono16") {
      return CV_16SC1;
    } else if (encoding == "rgba8") {
      return CV_8UC4;
    }
    throw std::runtime_error("Unsupported mat type");
  }

}  // namespace

namespace vslam_components {

  namespace vslam_nodes {

    IndirectVSlamNode::IndirectVSlamNode(const rclcpp::NodeOptions &options)
        : Node("vslam_node", options), K_{load_camera_info()} {
      // Feature extractor
      feature_extractor_ = feature_extractor_loader_.createSharedInstance(
          declare_parameter("feature_extractor_plugin_name", "UNDEFINED"));
      feature_extractor_->initialize(declare_parameter("num_features", 2000));

      // Feature matcher
      feature_matcher_
          = feature_matcher_loader_.createSharedInstance(declare_parameter("feature_matcher_plugin_name", "UNDEFINED"));
      feature_matcher_->initialize();

      // Camera tracker
      camera_tracker_
          = camera_tracker_loader_.createSharedInstance(declare_parameter("camera_tracker_plugin_name", "UNDEFINED"));
      camera_tracker_->initialize(K_);

      // Mapper
      mapper_ = mapper_loader_.createSharedInstance(declare_parameter("mapper_plugin_name", "UNDEFINED"));
      mapper_->initialize(K_);

      // Frame subscriber and publisher
      frame_sub_ = create_subscription<vslam_msgs::msg::Frame>("in_frame", 10,
                                                               std::bind(&IndirectVSlamNode::frame_callback, this, _1));
      frame_pub_ = create_publisher<vslam_msgs::msg::Frame>("out_frame", 10);
      captured_frame_pub_ = frame_pub_;
    }

    cv::Mat IndirectVSlamNode::load_camera_info() {
      double fx = declare_parameter("camera_params.fx", -1.0);
      double fy = declare_parameter("camera_params.fy", -1.0);
      double cx = declare_parameter("camera_params.cx", -1.0);
      double cy = declare_parameter("camera_params.cy", -1.0);

      if (fx == -1.0 || fy == -1.0 || cx == -1.0 || cy == -1.0) {
        throw std::runtime_error("Camera matrix not loaded!");
      }

      return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    void IndirectVSlamNode::frame_callback(vslam_msgs::msg::Frame::UniquePtr frame_msg) {
      auto pub_ptr = captured_frame_pub_.lock();
      if (!pub_ptr) {
        RCLCPP_WARN(this->get_logger(), "unable to lock the publisher\n");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Getting frame %u", frame_msg->id);

      // Create a cv::Mat from the image message (without copying).
      cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width, encoding2mat_type(frame_msg->image.encoding),
                     frame_msg->image.data.data());

      // Extract features in the image
      auto points = feature_extractor_->extract_features(cv_mat);

      switch (state_) {
        case State::init:
          current_keyframe_ = std::make_shared<vslam_datastructure::Frame>();
          current_keyframe_->from_msg(frame_msg.get());
          current_keyframe_->set_points(points);

          state_ = State::attempt_init;
          break;

        case State::attempt_init:
          if (!current_keyframe_->has_points() || points.empty()) {
            state_ = State::init;
            return;
          }

          auto current_frame = std::make_shared<vslam_datastructure::Frame>();
          current_frame->from_msg(frame_msg.get());
          current_frame->set_points(points);

          auto matched_points
              = feature_matcher_->match_features(current_keyframe_->get_points(), current_frame->get_points());

          const auto T_c_p = camera_tracker_->track_camera_2d2d(matched_points);

          // Camera pose
          cv::Mat T_p_w = current_keyframe_->get_pose();
          cv::Mat T_c_w = T_c_p * T_p_w;
          current_frame->set_pose(T_c_w);

          const auto new_mps = mapper_->map(matched_points, T_p_w, T_c_p);

          // write pose to the frame message
          constexpr bool skip_loaded = true;
          current_frame->to_msg(frame_msg.get(), skip_loaded);

          current_keyframe_ = current_frame;
          break;
      }

      pub_ptr->publish(std::move(frame_msg));
    }
  }  // namespace vslam_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::vslam_nodes::IndirectVSlamNode)