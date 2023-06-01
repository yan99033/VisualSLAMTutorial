#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {

  long unsigned int MapPoint::point_count_ = 0;

  void MapPoint::copy_from(MapPoint* other) {
    if (this == other || other == nullptr) {
      return;
    }
    // Lock both mutexes
    std::lock(this->mutex_, other->mutex_);

    // Use std::adopt_lock to tell std::lock_guard that the mutexes are already locked
    std::lock_guard<std::mutex> lock1(this->mutex_, std::adopt_lock);
    std::lock_guard<std::mutex> lock2(other->mutex_, std::adopt_lock);

    this->pt_3d_ = other->pt_3d_;

    this->is_outlier_.store(other->is_outlier_.load());

    std::swap(this->projections_, other->projections_);

    this->id_ = other->id_;
  }

  void MapPoint::set_pos(const cv::Point3d& pt_3d) {
    std::lock_guard<std::mutex> lck(mutex_);
    pt_3d_ = pt_3d;
  }

  cv::Point3d MapPoint::pos() {
    std::lock_guard<std::mutex> lck(mutex_);
    return pt_3d_;
  }

  void MapPoint::add_projection(Point* point) {
    std::lock_guard<std::mutex> lck(mutex_);
    projections_.insert(point);
  }

  void MapPoint::remove_projection(Point* point) {
    std::lock_guard<std::mutex> lck(mutex_);
    if (projections_.find(point) == projections_.end()) {
      return;
    }

    projections_.erase(point);
  }

  void MapPoint::set_host_keyframe_id(const long unsigned int host_keyframe_id) {
    std::lock_guard<std::mutex> lck(mutex_);
    host_keyframe_id_ = host_keyframe_id;
  }

  bool MapPoint::is_host(const long unsigned int keyframe_id) {
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

  void Point::set_frame(Frame* frame) {
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

  void Point::set_mappoint(MapPoint::SharedPtr mappoint) {
    std::lock_guard<std::mutex> lck(mutex_);
    mappoint_ = mappoint;
  }

  bool Point::has_mappoint() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (mappoint_.get() && !mappoint_->is_outlier()) {
      return true;
    } else {
      return false;
    }
  }

  bool Point::has_frame() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (frame_) {
      return true;
    } else {
      return false;
    }
  }

  bool Point::is_mappoint_host() {
    std::lock_guard<std::mutex> lck(mutex_);
    if (mappoint_ && frame_) {
      if (mappoint_->is_host(frame_->id())) {
        return true;
      }
    }
    return false;
  }
}  // namespace vslam_datastructure