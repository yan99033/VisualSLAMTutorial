#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {
  long unsigned int Point::point_count_ = 0;

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

  void MapPoint::set_mappoint(const cv::Point3d& pt_3d) {
    std::lock_guard<std::mutex> lck(mutex_);
    pt_3d_ = pt_3d;
  }

  void MapPoint::update_mappoint(const cv::Point3d& pt_3d) {
    std::lock_guard<std::mutex> lck(mutex_);
    pt_3d_ = (pt_3d_ + pt_3d) / 2;
  }

  cv::Point3d MapPoint::get_mappoint() {
    std::lock_guard<std::mutex> lck(mutex_);
    return pt_3d_;
  }

  void MapPoint::add_projection(Point* point) {
    std::lock_guard<std::mutex> lck(mutex_);
    projections_.insert(point);
  }

  void MapPoint::set_host_keyframe_id(const long unsigned int host_keyframe_id) {
    std::lock_guard<std::mutex> lck(mutex_);
    host_keyframe_id_ = host_keyframe_id;
  }

  bool MapPoint::is_host(const long unsigned int host_keyframe_id) {
    std::lock_guard<std::mutex> lck(mutex_);

    if (!host_keyframe_id_.has_value()) {
      return false;
    }

    if (host_keyframe_id_.value() == host_keyframe_id) {
      return true;
    } else {
      return false;
    }
  }

}  // namespace vslam_datastructure