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

#ifndef VSLAM_DATASTRUCTURE__TYPEDEFS_HPP_
#define VSLAM_DATASTRUCTURE__TYPEDEFS_HPP_

#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <set>
#include <unordered_set>
#include <vector>

namespace vslam_datastructure {
  // Forward declarations
  class Point;
  class MapPoint;
  class Frame;
  struct MatchedPoint;
  template <typename T> class SignalQueue;

  using PointSharedPtr = std::shared_ptr<Point>;
  using PointWeakPtr = std::weak_ptr<Point>;
  using Points = std::vector<PointSharedPtr>;
  using PointsPair = std::pair<Points, Points>;
  using MapPointSharedPtr = std::shared_ptr<MapPoint>;
  using MapPoints = std::vector<MapPointSharedPtr>;
  using PointMappointPair = std::pair<PointSharedPtr, MapPointSharedPtr>;
  using PointMappointPairs = std::vector<PointMappointPair>;
  using MatchedPoints = std::vector<MatchedPoint>;
  using MatchedIndexPairs = std::vector<std::pair<size_t, size_t>>;
  using FrameSharedPtr = std::shared_ptr<Frame>;
  using PointFramePair = std::pair<PointSharedPtr, FrameSharedPtr>;
  using FrameQueue = SignalQueue<FrameSharedPtr>;
  using Point3dPairs = std::vector<std::pair<cv::Point3d, cv::Point3d>>;
  using KeyPoints = std::vector<cv::KeyPoint>;

  struct PointFramePairHash {
    size_t operator()(const PointFramePair& point_frame_pair) const noexcept;
  };

  struct FrameCmp {
    bool operator()(const FrameSharedPtr& lhs, const FrameSharedPtr& rhs) const noexcept;
  };

  using FrameSet = std::unordered_set<FrameSharedPtr>;
  using PointSet = std::unordered_set<PointSharedPtr>;
  using PointFramePairSet
      = std::unordered_set<vslam_datastructure::PointFramePair, vslam_datastructure::PointFramePairHash>;
  using CoreKfsSet = std::set<FrameSharedPtr, FrameCmp>;
  using CoreMpsSet = std::unordered_set<MapPointSharedPtr>;
  using ProjectionSet = std::set<PointWeakPtr, std::owner_less<PointWeakPtr>>;

  /// Sim3 constraint between two keyframes
  struct Sim3Constraint {
    /// First keyframe
    FrameSharedPtr keyframe1;

    /// Second keyframe
    FrameSharedPtr keyframe2;

    /// Relative transformation from the first to the second keyframe
    cv::Mat T_1_2;

    /// Similarity transform scale
    double scale;
  };

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__TYPEDEFS_HPP_