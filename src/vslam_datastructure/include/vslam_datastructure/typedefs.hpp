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

#include <memory>
#include <set>
#include <vector>

namespace vslam_datastructure {
  // Forward declarations
  class Point;
  class MapPoint;
  class Frame;
  struct MatchedPoint;
  template <typename T> class SignalQueue;

  using PointSharedPtr = std::shared_ptr<Point>;
  using Points = std::vector<PointSharedPtr>;
  using MapPointSharedPtr = std::shared_ptr<MapPoint>;
  using MapPoints = std::vector<MapPointSharedPtr>;
  using PointMappointPairs = std::vector<std::pair<PointSharedPtr, MapPointSharedPtr>>;
  using MatchedPoints = std::vector<MatchedPoint>;
  using MatchedIndexPairs = std::vector<std::pair<size_t, size_t>>;
  using PointMappointPairs = std::vector<std::pair<PointSharedPtr, MapPointSharedPtr>>;
  using FrameSharedPtr = std::shared_ptr<Frame>;

  template <typename T> struct Cmp {
    bool operator()(const T* lhs, const T* rhs) const { return lhs->id() < rhs->id(); }
  };
  using CoreKfsSet = std::set<Frame*, Cmp<Frame>>;
  using CoreMpsSet = std::set<MapPoint*, Cmp<MapPoint>>;

  struct PointCmp {
    bool operator()(Point* const lhs, Point* const rhs) const;
  };
  using ProjectionSet = std::set<Point*, PointCmp>;

  using FrameQueue = SignalQueue<FrameSharedPtr>;

}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__TYPEDEFS_HPP_