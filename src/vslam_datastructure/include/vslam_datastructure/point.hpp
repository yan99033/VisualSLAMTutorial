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

  struct PointCmp {
    bool operator()(Point* const lhs, Point* const rhs) const;
  };

  class MapPoint {
  public:
    MapPoint() = default;

    using ProjectionSet = std::set<Point*, PointCmp>;

    using SharedPtr = std::shared_ptr<MapPoint>;

    /// Set the position of the map point
    /**
     * \param[in] pos_3d position of the map point
     */
    void setPos(const cv::Point3d& pos_3d);

    /// Get the position of the map point
    /**
     * \return position of the map point
     */
    cv::Point3d pos();

    /// Add a projection from `this` map point to a point in a frame
    /**
     * \param[in] point add a new projection from the map point to the point in a frame
     */
    void addProjection(Point* point);

    /// Remove the projection of the point
    /**
     * \param[in] point remove the existing projection from the map point to the point in a frame
     */
    void removeProjection(Point* point);

    /// Return a copy of the projections
    /**
     * It will go through the projections and remove any stale projections (i.e., the null pointers in the set)
     *
     * \return projections
     */
    ProjectionSet projections();

    /// Make the map point an inlier
    inline void setInlier() { isOutlier_ = false; };

    /// Make the map point an outlier
    inline void setOutlier() { isOutlier_ = true; }

    /// Check if the map point is an outlier
    /**
     * \return a boolean indicating if the map point is an outlier
     */
    inline bool isOutlier() const { return isOutlier_; }

    /// Map point id
    /**
     * \return map point id
     */
    inline long unsigned int id() const { return id_; }

    /// Set the host keyframe id
    /**
     * \param[in] host_keyframe_id the host keyframe id of this map point
     */
    void setHostKeyframeId(const long unsigned int host_keyframe_id);

    /// Check if the keyframe id matches the map point host keyframe id
    /**
     *  \param keyframe_id a keyframe id
     *  \return a boolean indicating if the id matches the host keyframe id
     */
    bool isHost(const long unsigned int keyframe_id);

    /// Check if the map point is associated with a host keyframe id
    /**
     * \return a boolean indicating if the map point is associated with a host keyframe id
     */
    inline bool hasHost() const { return host_keyframe_id_.has_value(); }

  private:
    /// Mutex for synchronizing reading and writing mappoint data
    std::mutex mutex_;

    /// 3D position of the map point
    cv::Point3d pos_3d_;

    /// A flag to indicate if the map point is an outlier
    std::atomic_bool isOutlier_{false};

    /// The corresponding points in the keyframes
    ProjectionSet projections_;

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

    /// Destructor
    ~Point() noexcept;

    /// keypoint (unprotected; use responsibly)
    cv::KeyPoint keypoint;

    /// Descriptor (unprotected; use responsibly)
    cv::Mat descriptor;

    /// Type (unprotected; use responsibly)
    Type type{Type::undefined};

    /// Set the frame pointer (non-owning) of the point
    /**
     * \param[in] frame frame pointer
     */
    void setFrame(Frame* frame);

    /// Get the frame pointer
    Frame* frame();

    /// Get the map point of the point
    MapPoint::SharedPtr mappoint();

    /// Set the map point of the point
    /**
     * \param[in] mappoint Shared pointer to a map point
     */
    void setMappoint(MapPoint::SharedPtr mappoint);

    /// Delete the map point of the point
    void deleteMappoint();

    /// Check if there is a map point associated with the point
    /**
     * \return A boolean indicating if there is a map point
     */
    bool hasMappoint();

    /// Check if there is a frame associated with the point
    /**
     * \return A boolean indicating if there is a frame
     */
    bool hasFrame();

    /// A boolean indicating if the host of the map point (if there is one associated) is 'this' point
    /**
     * \return A boolean indicating if the this point host the map point
     */
    bool isMappointHost();

  private:
    /// Map point of this point
    MapPoint::SharedPtr mappoint_;

    /// The frame of this point
    Frame* frame_;

    /// Point id
    long unsigned int id{point_count_++};

    /// Number of points so far
    static long unsigned int point_count_;

    /// Mutex for reading and writing point data
    std::mutex mutex_;
  };
  using Points = std::vector<Point::SharedPtr>;

  /// Point correspondence
  struct MatchedPoint {
    /// Point in frame 1
    Point::SharedPtr point1;

    /// Point in frame 2
    Point::SharedPtr point2;
  };
  using MatchedPoints = std::vector<MatchedPoint>;
  using MatchedIndexPairs = std::vector<std::pair<size_t, size_t>>;
  using Matches = std::pair<MatchedPoints, MatchedIndexPairs>;
}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__POINT_HPP_
