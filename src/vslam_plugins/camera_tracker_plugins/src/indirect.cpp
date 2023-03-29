#include "vslam_camera_tracker_plugins/indirect.hpp"

#include <cmath>
#include <iostream>

namespace {
  using cvPoint2dVec = std::vector<cv::Point2d>;
  std::pair<cvPoint2dVec, cvPoint2dVec> getCorrespondences(const vslam_datastructure::MatchedPoints& matched_points) {
    cvPoint2dVec cv_points1;
    cvPoint2dVec cv_points2;
    for (const auto& match : matched_points) {
      cv_points1.push_back(match.point1->keypoint.pt);
      cv_points2.push_back(match.point2->keypoint.pt);
    }

    return {cv_points1, cv_points2};
  }

  cv::Mat toTransformationMatrix(const cv::Mat& R, const cv::Mat& t) {
    // Create an identity matrix
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    // Copy data
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
  }
}  // namespace

namespace vslam_camera_tracker_plugins {
  void Indirect::initialize(const cv::Mat& K) { this->K = K; }

  cv::Mat Indirect::track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points) {
    const auto [cv_points1, cv_points2] = getCorrespondences(matched_points);

    cv::Mat inlier_mask;
    cv::Mat R;
    cv::Mat t;
    cv::Mat E = cv::findEssentialMat(cv_points1, cv_points2, K, cv::RANSAC, 0.999, 1.0, inlier_mask);
    const int num_inliers = cv::recoverPose(E, cv_points1, cv_points2, K, R, t, inlier_mask);

    return toTransformationMatrix(R, t);
  }

}  // namespace vslam_camera_tracker_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_camera_tracker_plugins::Indirect, vslam_camera_tracker_base::CameraTracker)