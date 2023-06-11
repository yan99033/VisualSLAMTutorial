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
    std::lock_guard<std::mutex> lck(mutex_);
    projections_.insert(point);
  }

  void MapPoint::removeProjection(Point* point) {
    std::lock_guard<std::mutex> lck(mutex_);
    if (projections_.find(point) == projections_.end()) {
      return;
    }

    projections_.erase(point);
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