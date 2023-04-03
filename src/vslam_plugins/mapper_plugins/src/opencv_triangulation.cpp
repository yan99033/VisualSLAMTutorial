#include "vslam_mapper_plugins/opencv_triangulation.hpp"

#include <cmath>
#include <iostream>

using PointIdx = std::vector<size_t>;

namespace {
  std::tuple<cv::Mat, cv::Mat, PointIdx> getCorrespondences(const vslam_datastructure::MatchedPoints& matched_points) {
    const size_t npts = matched_points.size();
    cv::Mat points1_mat = cv::Mat(2, npts, CV_64F);
    cv::Mat points2_mat = cv::Mat(2, npts, CV_64F);
    PointIdx match_idx;

    for (size_t i = 0; i < matched_points.size(); i++) {
      if (!matched_points[i].point1->mappoint.get()) {
        points1_mat.at<double>(0, i) = matched_points[i].point1->keypoint.pt.x;
        points1_mat.at<double>(1, i) = matched_points[i].point1->keypoint.pt.y;
        points2_mat.at<double>(0, i) = matched_points[i].point2->keypoint.pt.x;
        points2_mat.at<double>(1, i) = matched_points[i].point2->keypoint.pt.y;
        match_idx.push_back(i);
      }
    }

    return {points1_mat, points2_mat, match_idx};
  }

  cv::Mat project_point_3d2d(const cv::Mat& pt, const cv::Mat& K) {
    cv::Mat pt_projected = pt.clone();
    pt_projected = K * pt_projected;
    pt_projected /= pt_projected.at<double>(2, 0);

    return pt_projected.rowRange(0, 2);
  }

  double find_median_depth(const cv::Mat& mat) {
    // process the non-zero values only
    cv::Mat mat_cpy = mat.clone();
    mat_cpy = cv::max(mat_cpy, cv::Scalar(0));
    const int nonzeros_count = cv::countNonZero(mat_cpy);
    const int half_size = nonzeros_count / 2;

    // Copy elements to vector
    std::vector<double> sorted(mat_cpy.rows * mat_cpy.cols * mat_cpy.channels());
    assert(mat_cpy.isContinuous());
    sorted.assign(mat_cpy.ptr<double>(0), mat_cpy.ptr<double>(0) + mat_cpy.total());

    // Calculate the median of the non-zero values
    std::nth_element(sorted.begin(), sorted.begin() + half_size - 1, sorted.end(), std::greater<double>());

    return sorted.at(half_size);
  }
}  // namespace

namespace vslam_mapper_plugins {
  void OpenCvTriangulation::initialize(const cv::Mat& K) { K_ = K; }

  vslam_datastructure::MapPoints OpenCvTriangulation::map(vslam_datastructure::MatchedPoints& matched_points,
                                                          const cv::Mat& T_1_w, const cv::Mat& T_2_1,
                                                          const bool normalize_depth) {
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

    // map scale
    const double pt_scale = [&]() {
      if (normalize_depth) {
        cv::Mat normalized_z = cv_points1_3d.row(2) / cv_points1_3d.row(3);
        return find_median_depth(normalized_z);
      } else {
        return 1.0;
      }
    }();

    // Create map points for the points
    vslam_datastructure::MapPoints new_mps;
    for (size_t i = 0; i < cv_points1.cols; i++) {
      // Normalize and transform to the global coordinates
      auto pt_3d = cv_points1_3d.col(i);
      pt_3d /= pt_3d.at<double>(3, 0);
      pt_3d /= pt_scale;

      // Remove points behind the camera
      if (pt_3d.at<double>(2) < 0.0) {
        continue;
      }

      // Remove points that have a large re-projection error
      cv::Mat pt_3d_projected = project_point_3d2d(pt_3d.rowRange(0, 3), K_);
      constexpr double proj_err_thresh = 8.0;
      if (cv::norm(pt_3d_projected, cv_points1.col(i)) > proj_err_thresh) {
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