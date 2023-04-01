#include "vslam_datastructure/frame.hpp"

#include <iostream>

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
}  // namespace

namespace vslam_datastructure {
  cv::Mat Frame::getPose() const { return T_f_w_; }

  void Frame::fromMsg(vslam_msgs::msg::Frame* frame_msg) {
    if (frame_msg == nullptr) {
      return;
    }

    id_ = frame_msg->id;

    timestamp_ = static_cast<double>(frame_msg->header.stamp.sec)
                 + static_cast<double>(frame_msg->header.stamp.nanosec) / 1000000000.0;

    cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width, encoding2mat_type(frame_msg->image.encoding),
                   frame_msg->image.data.data());
    image_ = cv_mat.clone();
  }

  void Frame::setPose(const cv::Mat& T_f_w) { T_f_w_ = T_f_w.clone(); }

  void Frame::setPoints(vslam_datastructure::Points& points) { points_.swap(points); }

  Points* Frame::getPoints() { return &points_; }

  bool Frame::hasPoints() const { return !points_.empty(); }
}  // namespace vslam_datastructure