#ifndef VSLAM_DATASTRUCTURE__POINT_HPP_
#define VSLAM_DATASTRUCTURE__POINT_HPP_

#include <opencv2/opencv.hpp>

#include "vslam_datastructure/frame.hpp"

namespace vslam_datastructure {
  // Forward declarations
  struct Point;
  class Frame;

  struct MapPoint {
    using SharedPtr = std::shared_ptr<MapPoint>;

    cv::Point3d pt_3d;

    bool is_outlier{false};

    /*
     * @brief: the corresponding points
     */
    std::set<Point*> projections;

    long unsigned int id{point_count++};

    static long unsigned int point_count;
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

    long unsigned int id{point_count++};

    static long unsigned int point_count;
  };
  using Points = std::vector<Point::SharedPtr>;

  struct MatchedPoint {
    Point::SharedPtr point1;
    Point::SharedPtr point2;
  };
  using MatchedPoints = std::vector<MatchedPoint>;
  using MatchedIndexPairs = std::vector<std::pair<size_t, size_t>>;

  struct Matches {
    MatchedPoints matched_points;
    MatchedIndexPairs matched_index_pairs;

    std::vector<size_t> get_first_indices() const;
    std::vector<size_t> get_second_indices() const;
  };
  // using Matches = std::pair<MatchedPoints, MatchedIndexPairs>;

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
