#include "vslam_datastructure/frame.hpp"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "vslam_datastructure/point.hpp"

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

  geometry_msgs::msg::Pose transformation_mat_to_pose_msg(const cv::Mat& T) {
    // Rotation matrix and translation vector
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    cv::Mat t = T(cv::Rect(3, 0, 1, 3));

    // Rotation matrix to quaternion
    cv::Mat rpy = cv::Mat::zeros(3, 1, CV_64F);
    cv::Rodrigues(R, rpy);
    tf2::Quaternion q;
    q.setRPY(rpy.at<double>(0), rpy.at<double>(1), rpy.at<double>(2));

    geometry_msgs::msg::Pose pose;
    pose.position.x = t.at<double>(0);
    pose.position.y = t.at<double>(1);
    pose.position.z = t.at<double>(2);
    pose.orientation = tf2::toMsg(q);

    return pose;
  }
}  // namespace

namespace vslam_datastructure {
  void Frame::from_msg(vslam_msgs::msg::Frame* frame_msg) {
    if (frame_msg == nullptr) {
      return;
    }

    id_ = frame_msg->id;

    timestamp_ = static_cast<double>(frame_msg->header.stamp.sec)
                 + static_cast<double>(frame_msg->header.stamp.nanosec) / 1000000000.0;
    ros_timestamp_sec_ = frame_msg->header.stamp.sec;
    ros_timestamp_nanosec_ = frame_msg->header.stamp.nanosec;

    cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width, encoding2mat_type(frame_msg->image.encoding),
                   frame_msg->image.data.data());
    image_ = cv_mat.clone();
  }

  void Frame::to_msg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded) const {
    if (frame_msg == nullptr) {
      return;
    }

    if (!skip_loaded) {
      frame_msg->id = id_;
      frame_msg->header.stamp.sec = ros_timestamp_sec_;
      frame_msg->header.stamp.nanosec = ros_timestamp_nanosec_;

      frame_msg->image.height = image_.rows;
      frame_msg->image.width = image_.cols;
      frame_msg->image.encoding = mat_type2encoding(image_.type());
      frame_msg->image.is_bigendian = false;
      frame_msg->image.step = static_cast<sensor_msgs::msg::Image::_step_type>(image_.step);
      frame_msg->image.data.assign(image_.datastart, image_.dataend);
    }

    frame_msg->pose = transformation_mat_to_pose_msg(T_f_w_.inv());

    for (const auto& pt : points_) {
      if (pt->mappoint.get() && !pt->mappoint->is_outlier) {
        // 2D keypoints
        vslam_msgs::msg::Vector2d pt_2d;
        pt_2d.x = pt->keypoint.pt.x;
        pt_2d.y = pt->keypoint.pt.y;
        frame_msg->keypoints.push_back(pt_2d);

        // 3D map points
        vslam_msgs::msg::Vector3d pt_3d;
        pt_3d.x = pt->mappoint->pt_3d.x;
        pt_3d.y = pt->mappoint->pt_3d.y;
        pt_3d.z = pt->mappoint->pt_3d.z;
        frame_msg->mappoints.push_back(pt_3d);
      }
    }
  }

  void Frame::set_points(vslam_datastructure::Points& points) {
    points_.swap(points);

    // Set the frame ptr
    for (auto& pt : points_) {
      pt->frame = this;
    }
  }

  size_t Frame::get_num_mps() const {
    size_t num_mps{0};
    for (const auto& pt : points_) {
      if (pt->mappoint.get() && !pt->mappoint->is_outlier) {
        num_mps++;
      }
    }
    return num_mps;
  }

  void Frame::set_keyframe() {
    set_mappoint_projections();
    is_keyframe_ = true;
  }

  void Frame::set_T_this_prev_kf(Frame::SharedPtr prev_kf, const cv::Mat& T_this_prev) {
    T_this_prev_kf_ = {prev_kf, T_this_prev};
  }

  void Frame::set_T_this_next_kf(Frame::SharedPtr next_kf, const cv::Mat& T_this_next) {
    T_this_next_kf_ = {next_kf, T_this_next};
  }

  void Frame::set_mappoint_projections() {
    for (auto& pt : points_) {
      if (pt->mappoint.get() && !pt->mappoint->is_outlier) {
        pt->mappoint->projections.insert(pt.get());
      }
    }
  }
}  // namespace vslam_datastructure