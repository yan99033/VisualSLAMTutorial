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

    /// Copy from another mappoint (TODO: do we still need it?)
    void copy_from(MapPoint* other);

    /// Set the position of the map point
    void set_pos(const cv::Point3d& pt_3d);

    /// Get the position of the map point
    /**
     * \return position of the map point
     */
    cv::Point3d get_pos();

    /// Add a projection from `this` map point to a point in a frame
    void add_projection(Point* point);

    /// Remove the projection of the point
    void remove_projection(Point* point);

    /// Get the projections
    /**
     * \note Do not modify the projections, i.e., calling MapPoint::add_projection or MapPoint::remove_projection while
     * iterating through the projections
     *
     * \return projections
     */
    inline const std::set<Point*>& get_projections() const { return projections_; }

    /// Make the map point an inlier
    inline void set_inlier() { is_outlier_ = false; };

    /// Make the map point an outlier
    inline void set_outlier() { is_outlier_ = true; }

    /// Check if the map point is an outlier
    /**
     * \return a boolean indicating if the map point is an outlier
     */
    inline bool is_outlier() const { return is_outlier_; }

    /// Map point id
    /**
     * \return map point id
     */
    inline long unsigned int id() const { return id_; }

    /// Set the host keyframe id
    void set_host_keyframe_id(const long unsigned int host_keyframe_id);

    /// Check if the keyframe id matches the map point host keyframe id
    /**
     *  \param keyframe_id a keyframe id
     *  \return a boolean indicating if the id matches the host keyframe id
     */
    bool is_host(const long unsigned int keyframe_id);

    /// Check if the map point is associated with a host keyframe id
    /**
     * \return a boolean indicating if the map point is associated with a host keyframe id
     */
    inline bool has_host() const { return host_keyframe_id_.has_value(); }

  private:
    /// Mutex for synchronizing reading and writing mappoint data
    std::mutex mutex_;

    /// 3D position of the map point
    cv::Point3d pt_3d_;

    /// A flag to indicate if the map point is an outlier
    std::atomic_bool is_outlier_{false};

    /// The corresponding points in the keyframes
    std::set<Point*> projections_;

    /// Point id
    long unsigned int id_{point_count_++};

    /// The total number of points created so far
    static long unsigned int point_count_;

    // The host keyframe id
    std::optional<long unsigned int> host_keyframe_id_{std::nullopt};
  };
  using MapPoints = std::vector<MapPoint::SharedPtr>;

  class Point {
  public:
    /// Point type
    enum class Type { undefined = 0, orb = 1 };

    using SharedPtr = std::shared_ptr<Point>;

    /// Remove the default constructor
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

    /// Set the frame pointer (non-owning) of the point
    /**
     * \param frame frame pointer
     */
    void set_frame(Frame* frame);

    /// Get the frame pointer
    Frame* get_frame();

    /// Get the map point of the point
    MapPoint::SharedPtr get_mappoint();

    /// Set the map point of the point
    void set_mappoint(MapPoint::SharedPtr mappoint);

    /// Check if there is a map point associated with the point
    /**
     * \return A boolean indicating if there is a map point
     */
    bool has_mappoint();

    /// Check if there is a frame associated with the point
    /**
     * \return A boolean indicating if there is a frame
     */
    bool has_frame();

    /// A boolean indicating if the host of the map point (if there is one associated) is 'this' point
    bool is_mappoint_host();

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
