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

#ifndef VSLAM_DATASTRUCTURE__UTILS_HPP_
#define VSLAM_DATASTRUCTURE__UTILS_HPP_

#include "vslam_datastructure/point.hpp"
#include "vslam_datastructure/typedefs.hpp"

namespace vslam_datastructure {
  namespace utils {
    /// Get the corresponding map points from the point correspondences
    /**
     * \param[in] matched_points point correspondences
     * \return a vector containing pairs of corresponding map points
     */
    Point3dPairs mappointCorrespondencesFromMatchedPoints(const vslam_datastructure::MatchedPoints& matched_points);

    /// Get the corresponding points and their map points from the matched points
    /**
     * This is used to replace the old map points with the new ones when a loop is found
     * \param[in] matched_points point correspondences
     * \return a vector containint the point-mappoint pairs
     */
    PointMappointPairs secondPointFirstMappointPairsFromMatchedPoints(const MatchedPoints& matched_points);

    /// Concatenate the keypoint descriptors into a matrix
    /**
     * \param points[in] a vector of points
     * \return a pair of type keypoints and descriptors
     */
    KeyPoints extractKeypointsFromPoints(const Points& points);

    /// Extract and concatenate descriptors from a vector of points
    /**
     * \param[in] points a vector of points
     * \return descriptors
     */
    cv::Mat extractDescriptorsFromPoints(const Points& points);

    /// Extract map points from a vector of points
    /**
     * \param[in] points a vector of points
     * \return map points
     */
    MapPoints extractMappointsFromPoints(const Points& points);

    /// Split the matched points into two vectors of the same length containing the corresponding points
    /**
     * \param[in] matched_points matched points where each match has the points in frame1 and frame2
     * \return a pair of vectors containing the corresponding points
     */
    PointsPair splitMatchedPoints(const vslam_datastructure::MatchedPoints& matched_points);

  }  // namespace utils
}  // namespace vslam_datastructure

#endif  // VSLAM_DATASTRUCTURE__UTILS_HPP_