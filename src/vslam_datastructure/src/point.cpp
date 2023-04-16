#include "vslam_datastructure/point.hpp"

namespace vslam_datastructure {
  long unsigned int Point::point_count = 0;

  long unsigned int MapPoint::point_count = 0;

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

}  // namespace vslam_datastructure