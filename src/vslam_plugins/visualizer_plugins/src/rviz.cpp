#include "vslam_visualizer_plugins/rviz.hpp"

#include <opencv2/opencv.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace {
  int encoding2mat_type(const std::string& encoding) {
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

  void add_keypoints_to_image_frame_msg(vslam_msgs::msg::Frame& frame_msg) {
    // Create a cv::Mat from the image message (without copying).
    cv::Mat cv_mat(frame_msg.image.height, frame_msg.image.width, encoding2mat_type(frame_msg.image.encoding),
                   frame_msg.image.data.data());

    // Add 2D keypoints to the image message and publish
    for (size_t i = 0; i < frame_msg.keypoints.size(); i++) {
      auto kp = frame_msg.keypoints.at(i);
      auto has_mp = frame_msg.keypoints_has_mp.at(i);

      constexpr int ft_radius = 5;
      const cv::Scalar ft_rgb(0, 0, 255);
      const int ft_thickness = -1;
      cv::circle(cv_mat, cv::Point2d{kp.x, kp.y}, ft_radius, ft_rgb, ft_thickness);

      if (has_mp) {
        constexpr int half_size = 5;
        const cv::Scalar sq_color(0, 255, 0);
        const int sq_thickness = 2;
        cv::rectangle(cv_mat, cv::Point2d{kp.x - half_size, kp.y - half_size},
                      cv::Point2d{kp.x + half_size, kp.y + half_size}, sq_color, sq_thickness);
      }
    }
  }

  visualization_msgs::msg::Marker calculate_pose_marker(const geometry_msgs::msg::Pose& pose,
                                                        const std::string& frame_id, const double scale,
                                                        const double line_thickness, const std::string& marker_ns,
                                                        const int marker_id, const std::array<double, 3>& rgb,
                                                        const Eigen::Matrix3d& cam_axes_transform,
                                                        const rclcpp::Duration& duration) {
    // Convert the pose to an Eigen matrix
    Eigen::Isometry3d eigen_transform;
    tf2::fromMsg(pose, eigen_transform);

    // Calculate the preset vertices
    constexpr double fx = 340.0;
    constexpr double fy = 350.0;
    constexpr double cx = 300.0;
    constexpr double cy = 200.0;
    constexpr double w = 600.0;
    constexpr double h = 400.0;
    const double s = scale;
    const double p0[] = {0.0, 0.0, 0.0, 1.0};
    const double p1[] = {s * (0 - cx) / fx, s * (0 - cy) / fy, s, 1.0};
    const double p2[] = {s * (0 - cx) / fx, s * (h - 1 - cy) / fy, s, 1.0};
    const double p3[] = {s * (w - 1 - cx) / fx, s * (h - 1 - cy) / fy, s, 1.0};
    const double p4[] = {s * (w - 1 - cx) / fx, s * (0 - cy) / fy, s, 1.0};

    constexpr int num_vertices = 16;
    constexpr int num_dims = 4;
    Eigen::MatrixXd marker_vertices(num_dims, num_vertices);
    marker_vertices << p0[0], p1[0], p0[0], p2[0], p0[0], p3[0], p0[0], p4[0], p4[0], p3[0], p3[0], p2[0], p2[0], p1[0],
        p1[0], p4[0],  //
        p0[1], p1[1], p0[1], p2[1], p0[1], p3[1], p0[1], p4[1], p4[1], p3[1], p3[1], p2[1], p2[1], p1[1], p1[1],
        p4[1],         //
        p0[2], p1[2], p0[2], p2[2], p0[2], p3[2], p0[2], p4[2], p4[2], p3[2], p3[2], p2[2], p2[2], p1[2], p1[2],
        p4[2],         //
        p0[3], p1[3], p0[3], p2[3], p0[3], p3[3], p0[3], p4[3], p4[3], p3[3], p3[3], p2[3], p2[3], p1[3], p1[3],
        p4[3];         //
    marker_vertices = eigen_transform.matrix() * marker_vertices;
    marker_vertices.topRows(3) = cam_axes_transform * marker_vertices.topRows(3);

    visualization_msgs::msg::Marker pose_marker;
    // Add vertices to the marker msg
    pose_marker.header.frame_id = frame_id;
    pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    pose_marker.action = visualization_msgs::msg::Marker::ADD;
    pose_marker.scale.x = line_thickness;
    pose_marker.color.r = rgb[0];
    pose_marker.color.g = rgb[1];
    pose_marker.color.b = rgb[2];
    pose_marker.color.a = 1.0;
    pose_marker.lifetime = duration;
    pose_marker.id = marker_id;
    pose_marker.ns = marker_ns;
    for (int i = 0; i < num_vertices; i++) {
      auto pt = geometry_msgs::msg::Point();
      pt.x = marker_vertices.col(i)(0);
      pt.y = marker_vertices.col(i)(1);
      pt.z = marker_vertices.col(i)(2);

      pose_marker.points.push_back(pt);
    }

    return pose_marker;
  }

  visualization_msgs::msg::Marker calculate_mappoints_marker(const std::vector<vslam_msgs::msg::Vector3d>& mps,
                                                             const std::string& frame_id, const double scale,
                                                             const std::string& marker_ns, const int marker_id,
                                                             const std::array<double, 3>& rgb,
                                                             const Eigen::Matrix3d& cam_axes_transform,
                                                             const rclcpp::Duration& duration) {
    visualization_msgs::msg::Marker mps_marker;
    mps_marker.header.frame_id = frame_id;
    mps_marker.type = visualization_msgs::msg::Marker::POINTS;
    mps_marker.action = visualization_msgs::msg::Marker::ADD;
    mps_marker.scale.x = scale;
    mps_marker.scale.y = scale;
    mps_marker.scale.z = scale;
    mps_marker.color.r = rgb[0];
    mps_marker.color.g = rgb[1];
    mps_marker.color.b = rgb[2];
    mps_marker.color.a = 1.0;
    mps_marker.id = marker_id;
    mps_marker.ns = marker_ns;
    for (const auto& mp : mps) {
      Eigen::Vector3d mp_eigen(mp.x, mp.y, mp.z);
      mp_eigen = cam_axes_transform * mp_eigen;

      auto pt = geometry_msgs::msg::Point();
      pt.x = mp_eigen.x();
      pt.y = mp_eigen.y();
      pt.z = mp_eigen.z();
      mps_marker.points.push_back(pt);
    }

    return mps_marker;
  }
}  // namespace

namespace vslam_visualizer_plugins {
  const Eigen::Matrix3d RViz::cam_axes_transform_
      = (Eigen::Matrix3d() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0).finished();

  RViz::~RViz() {}

  void RViz::initialize() {
    node_ = std::make_shared<rclcpp::Node>("visualizer_node", "vslam");

    image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("live_image", 1);
    live_frame_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("live_frame_marker", 10);
    keyframe_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("keyframe_marker", 10);
  }

  void RViz::add_live_frame(const vslam_msgs::msg::Frame& frame_msg) {
    auto frame_msg_cpy = frame_msg;
    add_keypoints_to_image_frame_msg(frame_msg_cpy);
    image_publisher_->publish(frame_msg_cpy.image);

    auto pose_marker = calculate_pose_marker(frame_msg_cpy.pose, frame_id_, 10.0, line_thickness_, "live", -1,
                                             {1, 0, 0}, cam_axes_transform_, rclcpp::Duration({10000000}));
    live_frame_publisher_->publish(std::move(pose_marker));
  }

  void RViz::add_keyframe(const vslam_msgs::msg::Frame& frame_msg) {
    auto pose_marker = calculate_pose_marker(frame_msg.pose, frame_id_, marker_scale_, line_thickness_, "keyframe_pose",
                                             frame_msg.id, {0, 1, 0}, cam_axes_transform_, rclcpp::Duration({0}));
    keyframe_publisher_->publish(std::move(pose_marker));

    auto mps_marker = calculate_mappoints_marker(frame_msg.mappoints, frame_id_, marker_scale_, "keyfame_mps",
                                                 frame_msg.id, {0, 0, 0}, cam_axes_transform_, rclcpp::Duration({0}));
    keyframe_publisher_->publish(std::move(mps_marker));
  }

  void RViz::remove_keyframe(const vslam_msgs::msg::Frame& frame_msg) {}

  void RViz::replace_all_keyframes(const FrameVec& frame_msgs) {}

}  // namespace vslam_visualizer_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_visualizer_plugins::RViz, vslam_visualizer_base::Visualizer)