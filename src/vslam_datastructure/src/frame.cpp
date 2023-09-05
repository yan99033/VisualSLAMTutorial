/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "vslam_datastructure/frame.hpp"

#include <cassert>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_utils/converter.hpp"

namespace {
  cv::Point2f projectPoint3d2d(const cv::Point3d& pt_3d, const cv::Mat& K, const cv::Mat& T_f_w) {
    cv::Mat pt_projected = (cv::Mat_<double>(3, 1) << pt_3d.x, pt_3d.y, pt_3d.z);
    pt_projected = K * (T_f_w.rowRange(0, 3).colRange(0, 3) * pt_projected + T_f_w.rowRange(0, 3).colRange(3, 4));
    pt_projected /= pt_projected.at<double>(2, 0);

    return {static_cast<float>(pt_projected.at<double>(0, 0)), static_cast<float>(pt_projected.at<double>(1, 0))};
  }

  geometry_msgs::msg::Pose transformationMatToPoseMsg(const cv::Mat& T) {
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
  Frame::~Frame() noexcept {
    // Release resources
    points_.clear();
  }

  cv::Mat Frame::T_f_w() const {
    std::lock_guard<std::mutex> lck(data_mutex_);
    return T_f_w_.clone();
  }

  cv::Mat Frame::T_w_f() const {
    std::lock_guard<std::mutex> lck(data_mutex_);
    return T_f_w_.inv();
  }

  cv::Mat Frame::K() const { return K_.clone(); }

  void Frame::updateSim3PoseAndMps(const cv::Mat& S_f_w, const cv::Mat& T_f_w) {
    cv::Mat S_w_f = S_f_w.inv();
    cv::Matx33d sR_w_f = S_w_f.rowRange(0, 3).colRange(0, 3);
    cv::Mat st_w_f = S_w_f.rowRange(0, 3).colRange(3, 4);
    cv::Point3d st_w_f_pt(st_w_f);

    cv::Matx33d old_R_f_w = T_f_w_.rowRange(0, 3).colRange(0, 3);
    cv::Mat old_t_f_w = T_f_w_.rowRange(0, 3).colRange(3, 4);
    cv::Point3d old_t_f_w_pt(old_t_f_w);

    std::lock_guard<std::mutex> lck(data_mutex_);

    // Iterate through the map points with this keyframe being the host and update their global position
    for (auto& pt : points_) {
      if (pt->hasMappoint() && pt->mappoint()->isHost(id_)) {
        // Transform to this frame
        auto mp = pt->mappoint()->pos();
        mp = old_R_f_w * mp + old_t_f_w_pt;

        // Transform to the new pos in the world coordinate frame
        mp = sR_w_f * mp + st_w_f_pt;
        pt->mappoint()->setPos(mp);
      }
    }

    T_f_w_ = T_f_w.clone();
  }

  void Frame::setPose(const cv::Mat& T_f_w) {
    std::lock_guard<std::mutex> lck(data_mutex_);
    T_f_w_ = T_f_w.clone();
  }

  void Frame::fromMsg(vslam_msgs::msg::Frame* const frame_msg) {
    if (frame_msg == nullptr) {
      return;
    }

    std::lock_guard<std::mutex> lck(data_mutex_);

    id_ = frame_msg->id;

    timestamp_ = static_cast<double>(frame_msg->header.stamp.sec)
                 + static_cast<double>(frame_msg->header.stamp.nanosec) / 1000000000.0;
    ros_timestamp_sec_ = frame_msg->header.stamp.sec;
    ros_timestamp_nanosec_ = frame_msg->header.stamp.nanosec;

    cv::Mat cv_mat(frame_msg->image.height, frame_msg->image.width,
                   vslam_utils::conversions::encodingToCvMatType(frame_msg->image.encoding),
                   frame_msg->image.data.data());
    image_ = cv_mat.clone();

    cv::Mat K(3, 3, CV_64FC1, frame_msg->cam_info.k.data());
    K_ = K.clone();
  }

  void Frame::toMsg(vslam_msgs::msg::Frame* frame_msg, const bool skip_loaded, const bool no_mappoints) {
    if (frame_msg == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lck(data_mutex_);

    frame_msg->id = id_;

    if (!skip_loaded) {
      frame_msg->header.stamp.sec = ros_timestamp_sec_;
      frame_msg->header.stamp.nanosec = ros_timestamp_nanosec_;

      frame_msg->image.height = image_.rows;
      frame_msg->image.width = image_.cols;
      frame_msg->image.encoding = vslam_utils::conversions::cvMatTypeToEncoding(image_.type());
      frame_msg->image.is_bigendian = false;
      frame_msg->image.step = static_cast<sensor_msgs::msg::Image::_step_type>(image_.step);
      frame_msg->image.data.assign(image_.datastart, image_.dataend);
    }

    frame_msg->pose = transformationMatToPoseMsg(T_f_w_.inv());
    for (const auto& pt : points_) {
      if (pt->hasMappoint()) {
        if (!no_mappoints) {
          // Only visualize and update the map points that belong to the host
          if (pt->mappoint()->isHost(id_)) {
            // 3D map points
            vslam_msgs::msg::Vector3d pt_3d;
            const auto mp_pt_3d = pt->mappoint()->pos();
            pt_3d.x = mp_pt_3d.x;
            pt_3d.y = mp_pt_3d.y;
            pt_3d.z = mp_pt_3d.z;
            frame_msg->mappoints.push_back(pt_3d);
          }
        }

        // 2D keypoints
        vslam_msgs::msg::Vector2d pt_2d;
        pt_2d.x = pt->keypoint.pt.x;
        pt_2d.y = pt->keypoint.pt.y;
        frame_msg->keypoints.push_back(pt_2d);
      }
    }
  }

  void Frame::setPoints(Points& points) {
    std::lock_guard<std::mutex> lck(data_mutex_);
    points_.swap(points);

    // Set the frame ptr
    size_t i = 0;
    for (auto& pt : points_) {
      pt->setFrame(this);

      point_map_[pt] = i++;
    }
  }

  void Frame::setMappoints(const MapPoints& mappoints, const Points& points, const bool set_host) {
    assert(is_keyframe_);
    assert(mappoints.size() == points.size());

    std::lock_guard<std::mutex> lck(data_mutex_);

    for (size_t idx = 0; idx < points.size(); idx++) {
      assert(point_map_.find(points[idx]) != point_map_.end());

      const auto i = point_map_[points[idx]];

      if (!mappoints[idx].get()) {
        continue;
      }

      // TODO: resolve the discrepancy if there is an existing map point
      if (!points_[i]->hasMappoint()) {
        // Create a new map point
        points_[i]->setMappoint(mappoints[idx]);

        if (set_host && !points_[i]->mappoint()->hasHost()) {
          points_[i]->mappoint()->setHostKeyframeId(id_);
        }
      }
    }

    setMappointProjections();
  }

  void Frame::fuseMappoints(const PointMappointPairs& point_mappoint_pairs) {
    assert(is_keyframe_);

    std::lock_guard<std::mutex> lck(data_mutex_);

    for (const auto& [pt, mappoint] : point_mappoint_pairs) {
      assert(mappoint != nullptr);

      if (!pt->hasMappoint()) {
        // Associate the new map point with the old point
        mappoint->addProjection(pt.get());
        pt->setMappoint(mappoint);
      } else {
        auto old_mappoint = pt->mappoint();

        for (auto old_pt : old_mappoint->projections()) {
          // Add new projections to the new map point
          mappoint->addProjection(old_pt);

          // Replace the old map point
          old_pt->setMappoint(mappoint);
        }
      }
    }
  }

  void Frame::setKeyframe() {
    std::lock_guard<std::mutex> lck(data_mutex_);
    is_keyframe_ = true;
  }

  cv::Point3d Frame::mappointWorldToCam(const cv::Point3d& world_pos) const {
    std::lock_guard<std::mutex> lck(data_mutex_);
    cv::Matx33d R = T_f_w_.rowRange(0, 3).colRange(0, 3);
    cv::Mat t = T_f_w_.rowRange(0, 3).colRange(3, 4);
    return R * world_pos + cv::Point3d(t);
  }

  cv::Point2f Frame::mappointCamToPixel(const cv::Point3d& cam_pos) const {
    cv::Mat pt_projected = (cv::Mat_<double>(3, 1) << cam_pos.x, cam_pos.y, cam_pos.z);
    pt_projected = K_ * pt_projected;
    pt_projected /= pt_projected.at<double>(2, 0);

    return {static_cast<float>(pt_projected.at<double>(0, 0)), static_cast<float>(pt_projected.at<double>(1, 0))};
  }

  bool Frame::isBad() const {
    std::lock_guard<std::mutex> lck(data_mutex_);
    return is_bad_;
  }

  void Frame::setBad() {
    std::lock_guard<std::mutex> lck(data_mutex_);

    is_bad_ = true;

    // Delete map points
    for (auto& pt : points_) {
      if (pt->hasMappoint() && pt->isMappointHost()) {
        pt->deleteMappoint();
      }
    }
  }

  void Frame::setMappointProjections() {
    for (auto& pt : points_) {
      if (pt->hasMappoint()) {
        // Project the map point to its 2D keypoint and see if it is an inlier
        const auto pt_2d = projectPoint3d2d(pt->mappoint()->pos(), K_, T_f_w_);

        // Add projection if the reprojection error is low
        if (cv::norm(pt_2d - pt->keypoint.pt) < max_reproj_err_thresh_) {
          pt->mappoint()->addProjection(pt.get());
        }
      }
    }
  }

}  // namespace vslam_datastructure