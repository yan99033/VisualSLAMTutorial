#include "vslam_mapper_plugins/opencv_triangulation.hpp"

#include <cmath>
#include <iostream>

namespace {
  using cvPoint2dVec = std::vector<cv::Point2d>;
  using PointIdx = std::vector<size_t>;
  std::tuple<cvPoint2dVec, cvPoint2dVec, PointIdx> getCorrespondences(
      const vslam_datastructure::MatchedPoints& matched_points) {
    cvPoint2dVec cv_points1;
    cvPoint2dVec cv_points2;
    PointIdx match_idx;
    size_t i = 0;
    for (const auto& match : matched_points) {
      // Check if a map point is already available for the pair
      if (!match.point1->mappoint.get()) {
        cv_points1.push_back(match.point1->keypoint.pt);
        cv_points2.push_back(match.point2->keypoint.pt);
        match_idx.push_back(i);
      }
      i++;
    }

    return {cv_points1, cv_points2, match_idx};
  }
}  // namespace

namespace vslam_mapper_plugins {
  void OpenCvTriangulation::initialize(const cv::Mat& K) { K_ = K; }

  vslam_datastructure::MapPoints OpenCvTriangulation::map(vslam_datastructure::MatchedPoints& matched_points,
                                                          const cv::Mat& T_1_w, const cv::Mat& T_2_1) {
    // Get the matched points that don't have a map point
    // e.g., calculate a std::vector<bool> mask to indicate which matched points do not have a 3D point
    const auto [cv_points1, cv_points2, match_idx] = getCorrespondences(matched_points);

    // Calculate the projection matrix
    cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
    P1 = K_ * P1.rowRange(0, 3).colRange(0, 4);
    cv::Mat P2 = K_ * T_2_1.rowRange(0, 3).colRange(0, 4);

    // Triangulate the map points
    cv::Mat cv_points1_3d;
    cv::triangulatePoints(P1, P2, cv_points1, cv_points2, cv_points1_3d);

    // Create map points for the points
    vslam_datastructure::MapPoints new_mps;
    for (size_t i = 0; i < cv_points1.size(); i++) {
      // Normalize and transform to the global coordinates
      auto pt_3d = cv_points1_3d.col(i);
      pt_3d /= pt_3d.at<double>(3, 0);

      // Remove points behind the camera
      if (pt_3d.at<double>(2) < 0.0) {
        continue;
      }

      // Transform to world coordinate
      pt_3d = T_1_w.inv() * pt_3d;

      // The corresponding index in `matched_points`
      const auto j = match_idx[i];

      vslam_datastructure::MapPoint::SharedPtr mp = std::make_shared<vslam_datastructure::MapPoint>();
      mp->pt_3d = cv::Point3d(pt_3d.at<double>(0), pt_3d.at<double>(1), pt_3d.at<double>(2));
      mp->projections.push_back(matched_points[j].point1);
      mp->projections.push_back(matched_points[j].point2);

      matched_points[j].point1->mappoint = mp;
      matched_points[j].point2->mappoint = mp;

      new_mps.push_back(mp);
    }

    return new_mps;
  }

}  // namespace vslam_mapper_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_mapper_plugins::OpenCvTriangulation, vslam_mapper_base::Mapper)