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

  cv::Point2f project_point_3d2d(const cv::Point3d& pt_3d, const cv::Mat& K, const cv::Mat& T_f_w) {
    cv::Mat pt_projected = (cv::Mat_<double>(3, 1) << pt_3d.x, pt_3d.y, pt_3d.z);
    pt_projected = K * (T_f_w.rowRange(0, 3).colRange(0, 3) * pt_projected + T_f_w.rowRange(0, 3).colRange(3, 4));
    pt_projected /= pt_projected.at<double>(2, 0);

    return {static_cast<float>(pt_projected.at<double>(0, 0)), static_cast<float>(pt_projected.at<double>(1, 0))};
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
  Frame::Frame(const cv::Mat& K) : K_(K) {}

  cv::Mat Frame::T_f_w() {
    std::lock_guard<std::mutex> lck(data_mutex_);
    return T_f_w_.clone();
  }

  void Frame::set_pose(const cv::Mat& T_f_w) {
    std::lock_guard<std::mutex> lck(data_mutex_);
    T_f_w_ = T_f_w.clone();
  }

  void Frame::from_msg(vslam_msgs::msg::Frame* frame_msg) {
    if (frame_msg == nullptr) {
      return;
    }

    std::lock_guard<std::mutex> lck(data_mutex_);

    id_ = frame_msg->id;

    timestamp_ = static_cast<double>(frame_msg->header.stamp.sec)
                 + static_cast<double>(frame_msg->header.stamp.nanosec) / 1000000000.0;
    ros_timestamp_sec_ = frame_msg->header.stamp.sec;
    ros_timestamp_nanosec_ = frame_msg->header.stamp.nanosec;

    cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width, encoding2mat_type(frame_msg->image.encoding),
                   frame_msg->image.data.data());
    image_ = cv_mat.clone();
  }

  void Frame::to_msg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded, const bool no_mappoints) {
    if (frame_msg == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lck(data_mutex_);

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
      if (pt->mappoint.get() && !pt->mappoint->is_outlier()) {  // && pt->mappoint->host_keyframe() == this) {

        frame_msg->keypoints_has_mp.push_back(true);
        if (!no_mappoints) {
          // // Only visualize the map points that belong to the host
          // // FIXME: not working at the moment
          // // An alternative solution is to create a vector of type MapPoint::SharedPtr
          // if (pt->mappoint->is_host(id_)) {
          //   // 3D map points
          //   vslam_msgs::msg::Vector3d pt_3d;
          //   const auto mp_pt_3d = pt->mappoint->get_mappoint();
          //   pt_3d.x = mp_pt_3d.x;
          //   pt_3d.y = mp_pt_3d.y;
          //   pt_3d.z = mp_pt_3d.z;
          //   frame_msg->mappoints.push_back(pt_3d);
          // }

          // 3D map points
          vslam_msgs::msg::Vector3d pt_3d;
          const auto mp_pt_3d = pt->mappoint->get_mappoint();
          pt_3d.x = mp_pt_3d.x;
          pt_3d.y = mp_pt_3d.y;
          pt_3d.z = mp_pt_3d.z;
          frame_msg->mappoints.push_back(pt_3d);
        }
      } else {
        frame_msg->keypoints_has_mp.push_back(false);
      }

      // 2D keypoints
      vslam_msgs::msg::Vector2d pt_2d;
      pt_2d.x = pt->keypoint.pt.x;
      pt_2d.y = pt->keypoint.pt.y;
      frame_msg->keypoints.push_back(pt_2d);
    }
  }

  void Frame::set_points(Points& points) {
    std::lock_guard<std::mutex> lck(data_mutex_);
    points_.swap(points);

    // Set the frame ptr
    for (auto& pt : points_) {
      pt->frame = this;
    }
  }

  MapPoints Frame::get_map_points(const std::vector<size_t> point_indices) {
    std::lock_guard<std::mutex> lck(data_mutex_);

    MapPoints mappoints;
    for (const auto i : point_indices) {
      if (points_.at(i)->mappoint.get() && !points_.at(i)->mappoint->is_outlier()) {
        mappoints.push_back(points_.at(i)->mappoint);
      } else {
        mappoints.push_back(nullptr);
      }
    }
    return mappoints;
  }

  void Frame::set_map_points(const MapPoints& mappoints, const std::vector<size_t> indices, const bool set_host) {
    assert(is_keyframe_);
    assert(mappoints.size() == indices.size());

    std::lock_guard<std::mutex> lck(data_mutex_);

    for (size_t idx = 0; idx < indices.size(); idx++) {
      const auto i = indices.at(idx);
      if (mappoints.at(idx).get() == nullptr) {
        continue;
      }

      if (points_.at(i)->mappoint.get() && !points_.at(i)->mappoint->is_outlier()) {
        // Average the results
        points_.at(i)->mappoint->update_mappoint(mappoints.at(idx)->get_mappoint());
      } else {
        // Create a new map point
        points_.at(i)->mappoint = mappoints.at(idx);

        if (set_host) {
          points_.at(i)->mappoint->set_host_keyframe_id(id_);
        }
      }
    }

    set_mappoint_projections();
  }

  size_t Frame::get_num_mps() {
    std::lock_guard<std::mutex> lck(data_mutex_);
    size_t num_mps{0};
    for (const auto& pt : points_) {
      if (pt->mappoint.get() && !pt->mappoint->is_outlier()) {
        num_mps++;
      }
    }
    return num_mps;
  }

  void Frame::set_keyframe() {
    std::lock_guard<std::mutex> lck(data_mutex_);
    is_keyframe_ = true;
  }

  void Frame::set_T_this_prev_kf(const Frame* const prev_kf, const cv::Mat& T_this_prev) {
    std::lock_guard<std::mutex> lck(data_mutex_);
    T_this_prev_kf_ = {prev_kf, T_this_prev};
  }

  void Frame::add_T_this_next_kf(const Frame* const next_kf, const cv::Mat& T_this_next) {
    std::lock_guard<std::mutex> lck(data_mutex_);
    T_this_next_kf_.emplace_back(next_kf, T_this_next);
  }

  void Frame::set_mappoint_projections() {
    for (auto& pt : points_) {
      if (pt->mappoint.get() && !pt->mappoint->is_outlier()) {
        // Project the map point to its 2D keypoint and see if it is an inlier
        const auto pt_2d = project_point_3d2d(pt->mappoint->get_mappoint(), K_, T_f_w_);

        // Add projection if the reprojection error is low
        if (cv::norm(pt_2d - pt->keypoint.pt) < max_reproj_err_) {
          pt->mappoint->add_projection(pt.get());
        }
      }
    }
  }

}  // namespace vslam_datastructure