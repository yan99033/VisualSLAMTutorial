#include "vslam_camera_tracker_plugins/indirect.hpp"

#include <cmath>
#include <iostream>

using cvPoint2dVec = std::vector<cv::Point2d>;
using cvPoint3dVec = std::vector<cv::Point3d>;
using PointPtrVec = std::vector<vslam_datastructure::MapPoint::SharedPtr>;

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

  std::tuple<PointPtrVec, cvPoint3dVec, cvPoint2dVec> get_3d2d_correspondences(
      const vslam_datastructure::MatchedPoints& matched_points) {
    cvPoint3dVec cv_points_3d_1;
    cvPoint2dVec cv_points_2d_2;
    PointPtrVec points_3d_1_ptr;
    for (const auto& match : matched_points) {
      if (match.point1->mappoint.get() && !match.point1->mappoint->is_outlier()) {
        assert(match.point1->frame != nullptr);

        // Transform the 3d point in point1 to local
        cv::Point3d pt_3d_1 = match.point1->mappoint->get_mappoint();

        cv::Mat T_1_w = match.point1->frame->T_f_w();
        cv::Matx33d R = T_1_w.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T_1_w.rowRange(0, 3).colRange(3, 4);
        pt_3d_1 = R * pt_3d_1 + cv::Point3d(t.at<double>(0), t.at<double>(1), t.at<double>(2));

        cv_points_3d_1.push_back(pt_3d_1);
        cv_points_2d_2.push_back(match.point2->keypoint.pt);
        points_3d_1_ptr.push_back(match.point1->mappoint);

        // Mark it as outlier first and decide later
        match.point1->mappoint->set_outlier();
      }
    }

    return {points_3d_1_ptr, cv_points_3d_1, cv_points_2d_2};
  }

  cv::Mat to_transformation_matrix(const cv::Mat& R, const cv::Mat& t) {
    // Create an identity matrix
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    // Copy data
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));

    return T;
  }

  cv::Mat project_point_3d2d(const cv::Mat& pt, const cv::Mat& K) {
    cv::Mat pt_projected = pt.clone();
    pt_projected = K * pt_projected;
    pt_projected /= pt_projected.at<double>(2, 0);

    return pt_projected.rowRange(0, 2);
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

    cv::Mat rpy;
    cv::Rodrigues(R, rpy);

    std::cout << "-----------------------" << std::endl;
    std::cout << "Nister 5-pt solution: " << std::endl;
    std::cout << "rotation: " << rpy.t() << std::endl;
    std::cout << "translation: " << t.t() << std::endl;
    std::cout << "-----------------------" << std::endl;

    return to_transformation_matrix(R, t);
  }

  cv::Mat Indirect::track_camera_3d2d(const vslam_datastructure::MatchedPoints& matched_points, cv::Mat T_2_1_init) {
    auto [points_3d_1_ptr, cv_points_3d_1, cv_points_2d_2] = get_3d2d_correspondences(matched_points);

    cv::Mat rpy;
    cv::Mat t;
    const bool use_extrinsic_guess = [&]() {
      if (T_2_1_init.empty()) {
        return false;
      } else {
        cv::Rodrigues(T_2_1_init.rowRange(0, 3).colRange(0, 3), rpy);
        t = T_2_1_init.rowRange(0, 3).colRange(3, 4);
        return true;
      }
    }();
    constexpr int num_iter = 100;
    constexpr float reproj_err_thresh = 8.0;
    constexpr double confidence = 0.99;
    cv::Mat inliers;

    std::cout << "points used for PnP: " << points_3d_1_ptr.size() << std::endl;

    cv::solvePnPRansac(cv_points_3d_1, cv_points_2d_2, K_, cv::Mat(), rpy, t, use_extrinsic_guess, num_iter,
                       reproj_err_thresh, confidence, inliers);

    std::cout << "-----------------------" << std::endl;
    std::cout << "pnp solution: " << std::endl;
    std::cout << "rotation: " << rpy.t() << std::endl;
    std::cout << "translation: " << t.t() << std::endl;
    std::cout << "-----------------------" << std::endl;

    cv::Mat R;
    cv::Rodrigues(rpy, R);

    for (size_t i = 0; i < inliers.total(); i++) {
      points_3d_1_ptr.at(inliers.at<int>(i))->set_inlier();
    }

    return to_transformation_matrix(R, t);
  }

}  // namespace vslam_camera_tracker_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_camera_tracker_plugins::Indirect, vslam_camera_tracker_base::CameraTracker)