#include "vslam_mapper_plugins/opencv_triangulation.hpp"

#include <cmath>
#include <iostream>

namespace {
  std::pair<cv::Mat, cv::Mat> getCorrespondences(const vslam_datastructure::MatchedPoints& matched_points) {
    const size_t npts = matched_points.size();
    cv::Mat points1_mat = cv::Mat(2, npts, CV_64F);
    cv::Mat points2_mat = cv::Mat(2, npts, CV_64F);

    for (size_t i = 0; i < matched_points.size(); i++) {
      points1_mat.at<double>(0, i) = matched_points[i].point1->keypoint.pt.x;
      points1_mat.at<double>(1, i) = matched_points[i].point1->keypoint.pt.y;
      points2_mat.at<double>(0, i) = matched_points[i].point2->keypoint.pt.x;
      points2_mat.at<double>(1, i) = matched_points[i].point2->keypoint.pt.y;
    }

    return {points1_mat, points2_mat};
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
                                                          const cv::Mat& T_1_w, const cv::Mat& T_2_1) {
    // Preprocess the matched points for triangulation
    const auto [cv_points1, cv_points2] = getCorrespondences(matched_points);

    // Calculate the projection matrix
    cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
    P1 = K_ * P1.rowRange(0, 3).colRange(0, 4);
    cv::Mat P2 = K_ * T_2_1.rowRange(0, 3).colRange(0, 4);

    // Triangulate the map points
    cv::Mat cv_points1_3d;
    cv::triangulatePoints(P1, P2, cv_points1, cv_points2, cv_points1_3d);

    // // map scale
    // if (normalize_depth) {
    //   cv::Mat normalized_z = cv_points1_3d.row(2) / cv_points1_3d.row(3);
    //   normalize_scale = 1.0 / find_median_depth(normalized_z);
    // }

    // Create map points for the points
    vslam_datastructure::MapPoints new_mps;
    for (size_t i = 0; i < matched_points.size(); i++) {
      // Normalize and transform to the global coordinates
      auto pt_3d = cv_points1_3d.col(i);
      pt_3d /= pt_3d.at<double>(3, 0);
      // pt_3d *= normalize_scale;

      // Replace NaNs with zeros so the point can be removed
      for (size_t j = 0; j < 3; j++) {
        if (std::isnan(pt_3d.at<double>(j, 0))) {
          pt_3d.at<double>(j, 0) = 0;
        }
      }

      // Remove points that have a large re-projection error
      cv::Mat pt_3d_projected = project_point_3d2d(pt_3d.rowRange(0, 3), K_);
      constexpr double proj_err_thresh = 8.0;
      if (cv::norm(pt_3d_projected, cv_points1.col(i)) > proj_err_thresh) {
        continue;
      }

      // Remove points behind the camera
      if (pt_3d.at<double>(2, 0) <= 0.0) {
        continue;
      }

      // Transform to world coordinate
      pt_3d = T_1_w.inv() * pt_3d;

      if (matched_points.at(i).point1->mappoint.get() && !matched_points.at(i).point1->mappoint->is_outlier) {
        // If there is an existing map point, calculate the mean
        auto mp = matched_points.at(i).point1->mappoint;
        mp->pt_3d = cv::Point3d((mp->pt_3d.x + pt_3d.at<double>(0, 0)) / 2, (mp->pt_3d.y + pt_3d.at<double>(1, 0)) / 2,
                                (mp->pt_3d.z + pt_3d.at<double>(2, 0)) / 2);
      } else {
        // Create a new map point
        vslam_datastructure::MapPoint::SharedPtr mp = std::make_shared<vslam_datastructure::MapPoint>();
        mp->pt_3d = cv::Point3d(pt_3d.at<double>(0, 0), pt_3d.at<double>(1, 0), pt_3d.at<double>(2, 0));

        matched_points.at(i).point1->mappoint = mp;
        matched_points.at(i).point2->mappoint = mp;

        new_mps.push_back(mp);
      }
    }

    return new_mps;
  }

}  // namespace vslam_mapper_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_mapper_plugins::OpenCvTriangulation, vslam_mapper_base::Mapper)