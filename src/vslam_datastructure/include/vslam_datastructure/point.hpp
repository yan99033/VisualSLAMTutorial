#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

#include <atomic>
#include <limits>
#include <opencv2/opencv.hpp>
#include <optional>

#include "vslam_datastructure/frame.hpp"

namespace vslam_datastructure {
  // Forward declarations
  struct Point;
  class Frame;

  class MapPoint {
  public:
    MapPoint() = default;

    using SharedPtr = std::shared_ptr<MapPoint>;

    // Copy from another mappoint
    void copy_from(MapPoint* other);

    void set_mappoint(const cv::Point3d& pt_3d);

    // Update the map point by getting the element-wise mean between the existing and new points
    void update_mappoint(const cv::Point3d& pt_3d);

    cv::Point3d get_mappoint();

    void add_projection(Point* point);

    inline const std::set<Point*>& get_projections() const { return projections_; }

    inline void set_inlier() { is_outlier_ = false; };

    inline void set_outlier() { is_outlier_ = true; }

    inline bool is_outlier() const { return is_outlier_; }

    inline long unsigned int id() const { return id_; }

    void set_host_keyframe_id(const long unsigned int host_keyframe_id);

    bool is_host(const long unsigned int host_keyframe_id);

    inline bool has_host() const { return host_keyframe_id_.has_value(); }

  private:
    std::mutex mutex_;

    cv::Point3d pt_3d_;

    std::atomic_bool is_outlier_{false};

    /*
     * @brief: the corresponding points
     */
    std::set<Point*> projections_;

    long unsigned int id_{point_count_++};

    static long unsigned int point_count_;

    // The host keyframe id
    std::optional<long unsigned int> host_keyframe_id_{std::nullopt};
  };
  using MapPoints = std::vector<MapPoint::SharedPtr>;

  struct Point {
    using SharedPtr = std::shared_ptr<Point>;

    enum class Type { undefined = 0, orb = 1 };

    cv::KeyPoint keypoint;

    cv::Mat descriptor;

    Type type{Type::undefined};

    MapPoint::SharedPtr mappoint;

    Frame* frame;  //!< non-owning frame pointer

    long unsigned int id{point_count_++};

    static long unsigned int point_count_;
  };
  using Points = std::vector<Point::SharedPtr>;

  struct MatchedPoint {
    Point::SharedPtr point1;
    Point::SharedPtr point2;
  };
  using MatchedPoints = std::vector<MatchedPoint>;
  using MatchedIndexPairs = std::vector<std::pair<size_t, size_t>>;
  using Matches = std::pair<MatchedPoints, MatchedIndexPairs>;
}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
