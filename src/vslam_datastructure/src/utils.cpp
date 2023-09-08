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

#include "vslam_datastructure/utils.hpp"

namespace vslam_datastructure {
  namespace utils {
    Point3dPairs mappointCorrespondencesFromMatchedPoints(const vslam_datastructure::MatchedPoints& matched_points) {
      Point3dPairs mappoint_pairs;

      for (const auto& match : matched_points) {
        if (match.point1->hasMappoint() && match.point2->hasMappoint()) {
          auto frame1 = match.point1->frame();
          auto frame2 = match.point2->frame();

          assert(frame1 && frame2);

          cv::Point3d mp1 = frame1->mappointWorldToCam(match.point1->mappoint()->pos());
          cv::Point3d mp2 = frame2->mappointWorldToCam(match.point2->mappoint()->pos());

          mappoint_pairs.emplace_back(std::make_pair(mp1, mp2));
        }
      }

      return mappoint_pairs;
    }

    PointMappointPairs secondPointFirstMappointPairsFromMatchedPoints(const MatchedPoints& matched_points) {
      vslam_datastructure::PointMappointPairs point_mappoint_pairs;

      for (const auto& [pt1, pt2] : matched_points) {
        if (!pt1->hasMappoint()) {
          continue;
        }

        // Skip if the point has a map point but it is not the host of the map point
        if (pt2->hasMappoint() && !pt2->isMappointHost()) {
          continue;
        }

        point_mappoint_pairs.emplace_back(std::make_pair(pt2, pt1->mappoint()));
      }

      return point_mappoint_pairs;
    }

    KeyPoints extractKeypointsFromPoints(const Points& points) {
      std::vector<cv::KeyPoint> keypoints;
      for (const auto& pt : points) {
        keypoints.push_back(pt->keypoint);
      }

      return keypoints;
    }

    cv::Mat extractDescriptorsFromPoints(const Points& points) {
      std::vector<cv::Mat> descriptors_vec;
      for (const auto& pt : points) {
        descriptors_vec.push_back(pt->descriptor);
      }
      cv::Mat descriptors;
      cv::vconcat(descriptors_vec, descriptors);

      return descriptors;
    }

    MapPoints extractMappointsFromPoints(const Points& points) {
      MapPoints mappoints;
      for (const auto& pt : points) {
        if (pt->hasMappoint()) {
          mappoints.push_back(pt->mappoint());
        } else {
          mappoints.push_back(nullptr);
        }
      }
      return mappoints;
    }

    PointsPair splitMatchedPoints(const vslam_datastructure::MatchedPoints& matched_points) {
      vslam_datastructure::Points pts1;
      vslam_datastructure::Points pts2;
      for (const auto& [pt1, pt2] : matched_points) {
        pts1.push_back(pt1);
        pts2.push_back(pt2);
      }
      return {pts1, pts2};
    }

  }  // namespace utils
}  // namespace vslam_datastructure