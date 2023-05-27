#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

#include <atomic>
#include <limits>
#include <opencv2/opencv.hpp>
#include <optional>

#include "vslam_datastructure/frame.hpp"

namespace vslam_datastructure {
  // Forward declarations
  class Point;
  class Frame;

  class MapPoint {
  public:
    MapPoint() = default;

    using SharedPtr = std::shared_ptr<MapPoint>;

    // Copy from another mappoint
    void copy_from(MapPoint* other);

    void set_pos(const cv::Point3d& pt_3d);

    cv::Point3d get_pos();

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

  class Point {
  public:
    enum class Type { undefined = 0, orb = 1 };

    using SharedPtr = std::shared_ptr<Point>;

    Point() = delete;

    /// Constructor
    /**
     * \param[in] keypoint keypoint (\sa https://docs.opencv.org/4.5.4/d2/d29/classcv_1_1KeyPoint.html)
     * \param[in] descriptor keypoint descriptor
     * \param[in] type point type
     */
    Point(const cv::KeyPoint& keypoint, const cv::Mat& descriptor = cv::Mat(), const Type type = Type::undefined);

    /// keypoint (unprotected; use responsibly)
    cv::KeyPoint keypoint;

    /// Descriptor (unprotected; use responsibly)
    cv::Mat descriptor;

    /// Type (unprotected; use responsibly)
    Type type{Type::undefined};

    void set_frame(Frame* frame);

    Frame* get_frame();

    MapPoint::SharedPtr get_mappoint();

    void set_mappoint(MapPoint::SharedPtr mappoint);

    bool has_mappoint();

    bool has_frame();

  private:
    MapPoint::SharedPtr mappoint_;

    Frame* frame_;  //!< non-owning frame pointer

    long unsigned int id{point_count_++};

    static long unsigned int point_count_;

    std::mutex mutex_;
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
