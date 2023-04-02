#include "vslam_camera_tracker_plugins/indirect.hpp"

#include <cmath>
#include <iostream>

using cvPoint2dVec = std::vector<cv::Point2d>;
using cvPoint3dVec = std::vector<cv::Point3d>;

namespace {
  std::pair<cvPoint2dVec, cvPoint2dVec> get_2d2d_correspondences(
      const vslam_datastructure::MatchedPoints& matched_points) {
    cvPoint2dVec cv_points_2d_1;
    cvPoint2dVec cv_points_2d_2;
    for (const auto& match : matched_points) {
      cv_points_2d_1.push_back(match.point1->keypoint.pt);
      cv_points_2d_2.push_back(match.point2->keypoint.pt);
    }

    return {cv_points_2d_1, cv_points_2d_2};
  }

  std::pair<cvPoint3dVec, cvPoint2dVec> get_3d2d_correspondences(
      const vslam_datastructure::MatchedPoints& matched_points) {
    cvPoint3dVec cv_points_3d_1;
    cvPoint2dVec cv_points_2d_2;
    for (const auto& match : matched_points) {
      if (match.point1->mappoint.get()) {
        assert(match.point1->frame != nullptr);

        // Transform the 3d point in point1 to local
        cv::Point3d pt_3d_1 = match.point1->mappoint->pt_3d;
        cv::Mat T_1_w = match.point1->frame->get_pose();
        cv::Matx33d R = T_1_w.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T_1_w.rowRange(0, 3).colRange(3, 4);
        pt_3d_1 = R * pt_3d_1 + cv::Point3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));

        cv_points_3d_1.push_back(pt_3d_1);
        cv_points_2d_2.push_back(match.point2->keypoint.pt);
      }
    }

    return {cv_points_3d_1, cv_points_2d_2};
  }

  cv::Mat to_transformation_matrix(const cv::Mat& R, const cv::Mat& t) {
    // Create an identity matrix
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    // Copy data
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
  }
}  // namespace

namespace vslam_camera_tracker_plugins {
  void Indirect::initialize(const cv::Mat& K) { K_ = K; }

  cv::Mat Indirect::track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points) {
    const auto [cv_points_2d_1, cv_points_2d_2] = get_2d2d_correspondences(matched_points);

    cv::Mat inlier_mask;
    cv::Mat R;
    cv::Mat t;
    cv::Mat E = cv::findEssentialMat(cv_points_2d_1, cv_points_2d_2, K_, cv::RANSAC, 0.999, 1.0, inlier_mask);
    const int num_inliers = cv::recoverPose(E, cv_points_2d_1, cv_points_2d_2, K_, R, t, inlier_mask);

    return to_transformation_matrix(R, t);
  }

  cv::Mat Indirect::track_camera_3d2d(const vslam_datastructure::MatchedPoints& matched_points) {
    const auto [cv_points_3d_1, cv_points_2d_2] = get_3d2d_correspondences(matched_points);

    cv::Mat rpy;
    cv::Mat t;
    cv::solvePnP(cv_points_3d_1, cv_points_2d_2, K_, cv::Mat(), rpy, t, false);

    cv::Mat R;
    cv::Rodrigues(rpy, R);

    return to_transformation_matrix(R, t);
  }

}  // namespace vslam_camera_tracker_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_camera_tracker_plugins::Indirect, vslam_camera_tracker_base::CameraTracker)