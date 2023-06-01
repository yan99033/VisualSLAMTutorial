#include "vslam_camera_tracker_plugins/indirect_tracker.hpp"

#include <cmath>
#include <iostream>

#include "vslam_utils/converter.hpp"

using CvPoint2dVec = std::vector<cv::Point2d>;
using CvPoint3dVec = std::vector<cv::Point3d>;
using PointPtrVec = std::vector<vslam_datastructure::MapPoint::SharedPtr>;

namespace {
  std::pair<CvPoint2dVec, CvPoint2dVec> get_2d2d_correspondences(
      const vslam_datastructure::MatchedPoints& matched_points) {
    CvPoint2dVec cv_points_2d_1;
    CvPoint2dVec cv_points_2d_2;
    for (const auto& match : matched_points) {
      cv_points_2d_1.push_back(match.point1->keypoint.pt);
      cv_points_2d_2.push_back(match.point2->keypoint.pt);
    }

    return {cv_points_2d_1, cv_points_2d_2};
  }

  std::tuple<PointPtrVec, CvPoint3dVec, CvPoint2dVec> get_3d2d_correspondences(
      const vslam_datastructure::MatchedPoints& matched_points) {
    CvPoint3dVec cv_points_3d_1;
    CvPoint2dVec cv_points_2d_2;
    PointPtrVec points_3d_1_ptr;
    for (const auto& match : matched_points) {
      if (match.point1->has_mappoint()) {
        assert(match.point1->has_frame());

        // Transform the 3d point in point1 to local
        cv::Point3d pt_3d_1 = match.point1->get_mappoint()->get_pos();

        cv::Mat T_1_w = match.point1->get_frame()->T_f_w();
        cv::Matx33d R = T_1_w.rowRange(0, 3).colRange(0, 3);
        cv::Mat t = T_1_w.rowRange(0, 3).colRange(3, 4);
        pt_3d_1 = R * pt_3d_1 + cv::Point3d(t);

        cv_points_3d_1.push_back(pt_3d_1);
        cv_points_2d_2.push_back(match.point2->keypoint.pt);
        points_3d_1_ptr.push_back(match.point1->get_mappoint());

        // Mark it as outlier first and decide later
        match.point1->get_mappoint()->set_outlier();
      }
    }

    return {points_3d_1_ptr, cv_points_3d_1, cv_points_2d_2};
  }

  cv::Mat project_point_3d2d(const cv::Mat& pt, const cv::Mat& K) {
    cv::Mat pt_projected = pt.clone();
    pt_projected = K * pt_projected;
    pt_projected /= pt_projected.at<double>(2, 0);

    return pt_projected.rowRange(0, 2);
  }
}  // namespace

namespace vslam_camera_tracker_plugins {

  bool IndirectTracker::track_camera_2d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                                          cv::Mat& T_2_1) {
    const auto [cv_points_2d_1, cv_points_2d_2] = get_2d2d_correspondences(matched_points);

    cv::Mat inlier_mask;
    cv::Mat R;
    cv::Mat t;
    cv::Mat E = cv::findEssentialMat(cv_points_2d_1, cv_points_2d_2, K, cv::RANSAC, find_ess_mat_prob_,
                                     find_ess_mat_thresh_, inlier_mask);
    const int num_inliers = cv::recoverPose(E, cv_points_2d_1, cv_points_2d_2, K, R, t, inlier_mask);

    cv::Mat rpy;
    cv::Rodrigues(R, rpy);

    std::cout << "-----------------------" << std::endl;
    std::cout << "Nister 5-pt solution: " << std::endl;
    std::cout << "rotation: " << rpy.t() << std::endl;
    std::cout << "translation: " << t.t() << std::endl;
    std::cout << "-----------------------" << std::endl;

    if (static_cast<double>(num_inliers) / static_cast<double>(matched_points.size()) < inlier_ratio) {
      return false;
    }

    T_2_1 = vslam_utils::conversions::to_transformation_matrix(R, t);

    return true;
  }

  bool IndirectTracker::track_camera_3d2d(const vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& K,
                                          cv::Mat& T_2_1) {
    auto [points_3d_1_ptr, cv_points_3d_1, cv_points_2d_2] = get_3d2d_correspondences(matched_points);

    cv::Mat rpy;
    cv::Mat t;

    cv::Mat inliers;
    cv::solvePnPRansac(cv_points_3d_1, cv_points_2d_2, K, cv::Mat(), rpy, t, use_extrinsic_guess_,
                       pnp_ransac_num_iters_, pnp_reproj_err_thresh_, pnp_confidence_, inliers);

    std::cout << "-----------------------" << std::endl;
    std::cout << "pnp solution: " << std::endl;
    std::cout << "rotation: " << rpy.t() << std::endl;
    std::cout << "translation: " << t.t() << std::endl;
    std::cout << "num inliers: " << inliers.total() << " / " << cv_points_3d_1.size() << std::endl;
    std::cout << "-----------------------" << std::endl;

    if (static_cast<double>(inliers.total()) / static_cast<double>(cv_points_3d_1.size()) < inlier_ratio) {
      return false;
    }

    cv::Mat R;
    cv::Rodrigues(rpy, R);

    for (cv::MatConstIterator_<int> it = inliers.begin<int>(); it != inliers.end<int>(); ++it) {
      points_3d_1_ptr.at(*it)->set_inlier();
    }

    T_2_1 = vslam_utils::conversions::to_transformation_matrix(R, t);

    return true;
  }

}  // namespace vslam_camera_tracker_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_camera_tracker_plugins::IndirectTracker, vslam_camera_tracker::base::CameraTracker)