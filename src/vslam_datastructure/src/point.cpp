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

#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {
  long unsigned int MapPoint::point_count_ = 0;

  void MapPoint::setPos(const cv::Point3d& pos_3d) {
    std::lock_guard<std::mutex> lck(mutex_);
    pos_3d_ = pos_3d;
  }

  cv::Point3d MapPoint::pos() {
    std::lock_guard<std::mutex> lck(mutex_);
    return pos_3d_;
  }

  void MapPoint::addProjection(Point* point) {
    // Ignore the null pointers
    if (!point) {
      return;
    }

    std::lock_guard<std::mutex> lck(mutex_);
    projections_.insert(point);
  }

  void MapPoint::removeProjection(Point* point) {
    // Ignore the null pointers
    if (!point) {
      return;
    }

    std::lock_guard<std::mutex> lck(mutex_);
    if (projections_.find(point) == projections_.end()) {
      return;
    }

    projections_.erase(point);
  }

  ProjectionSet MapPoint::projections() {
    // Remove the stale (nullptr) projections
    for (auto it = projections_.begin(); it != projections_.end();) {
      if (*it == nullptr) {
        std::lock_guard<std::mutex> lck(mutex_);
        it = projections_.erase(it);
      } else {
        ++it;
      }
    }

    // Create a copy of the projections
    return ProjectionSet(projections_);
  }

  void MapPoint::setHostKeyframeId(const long unsigned int host_keyframe_id) {
    std::lock_guard<std::mutex> lck(mutex_);
    host_keyframe_id_ = host_keyframe_id;
  }

  bool MapPoint::isHost(const long unsigned int keyframe_id) {
    std::lock_guard<std::mutex> lck(mutex_);

    if (!host_keyframe_id_.has_value() || host_keyframe_id_.value() != keyframe_id) {
      return false;
    } else {
      return true;
    }
  }

  long unsigned int Point::point_count_ = 0;

  Point::Point(const cv::KeyPoint& keypoint, const cv::Mat& descriptor, const Type type)
      : keypoint(keypoint), descriptor(descriptor), type(type) {}

  Point::~Point() noexcept {
    // Release resources
    mappoint_.reset();
  }

  void Point::setFrame(Frame* frame) {
    std::lock_guard<std::mutex> lck(mutex_);
    frame_ = frame;
  }

  Frame* Point::frame() {
    std::lock_guard<std::mutex> lck(mutex_);
    return frame_;
  }

  MapPoint::SharedPtr Point::mappoint() {
    std::lock_guard<std::mutex> lck(mutex_);
    return mappoint_;
  }

  void Point::setMappoint(MapPoint::SharedPtr mappoint) {
    std::lock_guard<std::mutex> lck(mutex_);
    mappoint_ = mappoint;
  }

  void Point::deleteMappoint() {
    if (!mappoint_) {
      return;
    }

    std::lock_guard<std::mutex> lck(mutex_);
    mappoint_.reset();
  }

  bool Point::hasMappoint() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (mappoint_.get() && !mappoint_->isOutlier()) {
      return true;
    } else {
      return false;
    }
  }

  bool Point::hasFrame() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (frame_) {
      return true;
    } else {
      return false;
    }
  }

  bool Point::isMappointHost() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (mappoint_ && frame_) {
      if (mappoint_->isHost(frame_->id())) {
        return true;
      }
    }
    return false;
  }
}  // namespace vslam_datastructure