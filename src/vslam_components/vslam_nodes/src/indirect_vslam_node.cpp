#include "vslam_nodes/indirect_vslam_node.hpp"

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

  std::pair<cv::Mat, cv::Mat> transformationMatToTranslationQuaternion(const cv::Mat &T) {
    // Rotation matrix and translation vector
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));

    // Rotation matrix to quaternion
    cv::Mat quat = cv::Mat::zeros(4, 1, CV_64F);
    cv::Rodrigues(R, quat);

    return {t, quat};
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

      if (!prev_points.empty()) {
        auto matched_points = feature_matcher_->match_features(prev_points, points);
        const auto T_c_p = camera_tracker_->track_camera_2d2d(matched_points);

        // Camera pose
        T_p_w_ = T_c_p * T_p_w_;

        const auto new_mps = mapper_->map(matched_points, T_p_w_, T_c_p);

        // write pose to the frame message
        const auto T_w_p = T_p_w_.inv();
        const auto [trans, quat] = transformationMatToTranslationQuaternion(T_w_p);
        frame_msg->pose.position.x = trans.at<double>(0);
        frame_msg->pose.position.y = trans.at<double>(1);
        frame_msg->pose.position.z = trans.at<double>(2);
        frame_msg->pose.orientation.x = quat.at<double>(0);
        frame_msg->pose.orientation.y = quat.at<double>(1);
        frame_msg->pose.orientation.z = quat.at<double>(2);
        frame_msg->pose.orientation.w = quat.at<double>(3);

        // Write 2D keypoints to the frame message
        for (const auto &[_, pt2] : matched_points) {
          vslam_msgs::msg::Vector2d pt_2d;
          pt_2d.x = pt2->keypoint.pt.x;
          pt_2d.y = pt2->keypoint.pt.y;
          frame_msg->keypoints.push_back(pt_2d);
        }

        // Write 3D map points to the frame message
        for (const auto &mp : new_mps) {
          vslam_msgs::msg::Vector3d pt_3d;
          pt_3d.x = mp->pt_3d.x;
          pt_3d.y = mp->pt_3d.y;
          pt_3d.z = mp->pt_3d.z;
          frame_msg->mappoints.push_back(pt_3d);
        }
      }

      prev_points = points;

      pub_ptr->publish(std::move(frame_msg));
    }
  }  // namespace vslam_nodes
}  // namespace vslam_components

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be
// discoverable when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vslam_components::vslam_nodes::IndirectVSlamNode)