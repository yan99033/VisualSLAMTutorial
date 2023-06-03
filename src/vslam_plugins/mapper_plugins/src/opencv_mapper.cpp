#include "vslam_mapper_plugins/opencv_mapper.hpp"

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

  // double find_median_depth(const cv::Mat& mat) {
  //   // process the non-zero values only
  //   cv::Mat mat_cpy = mat.clone();
  //   mat_cpy = cv::max(mat_cpy, cv::Scalar(0));
  //   const int nonzeros_count = cv::countNonZero(mat_cpy);
  //   const int half_size = nonzeros_count / 2;

  //   // Copy elements to vector
  //   std::vector<double> sorted(mat_cpy.rows * mat_cpy.cols * mat_cpy.channels());
  //   assert(mat_cpy.isContinuous());
  //   sorted.assign(mat_cpy.ptr<double>(0), mat_cpy.ptr<double>(0) + mat_cpy.total());

  //   // Calculate the median of the non-zero values
  //   std::nth_element(sorted.begin(), sorted.begin() + half_size - 1, sorted.end(), std::greater<double>());

  //   return sorted.at(half_size);
  // }
}  // namespace

namespace vslam_mapper_plugins {
  vslam_datastructure::MapPoints OpenCVMapper::map(vslam_datastructure::MatchedPoints& matched_points,
                                                   const cv::Mat& T_1_w, const cv::Mat& T_2_1, const cv::Mat& K) {
    // Preprocess the matched points for triangulation
    const auto [cv_points1, cv_points2] = getCorrespondences(matched_points);

    // Calculate the projection matrix
    cv::Mat P1 = cv::Mat::eye(4, 4, CV_64F);
    P1 = K * P1.rowRange(0, 3).colRange(0, 4);
    cv::Mat P2 = K * T_2_1.rowRange(0, 3).colRange(0, 4);

    // Triangulate the map points
    cv::Mat cv_points1_3d;
    cv::triangulatePoints(P1, P2, cv_points1, cv_points2, cv_points1_3d);

    // Create map points for the points
    vslam_datastructure::MapPoints new_mps;
    for (size_t i = 0; i < matched_points.size(); i++) {
      // Normalize and transform to the global coordinates
      auto pt_3d = cv_points1_3d.col(i);
      pt_3d /= pt_3d.at<double>(3, 0);

      // Check if NaN exists
      bool nan_found{false};
      for (size_t j = 0; j < 3; j++) {
        if (std::isnan(pt_3d.at<double>(j, 0))) {
          nan_found = true;
          break;
        }
      }
      if (nan_found) {
        new_mps.push_back(nullptr);
        continue;
      }

      // Remove points that have a large re-projection error
      cv::Mat pt_3d_projected = project_point_3d2d(pt_3d.rowRange(0, 3), K);
      if (cv::norm(pt_3d_projected, cv_points1.col(i)) > proj_err_thresh_) {
        new_mps.push_back(nullptr);
        continue;
      }

      // Remove points behind the camera
      if (pt_3d.at<double>(2, 0) <= 0.0) {
        new_mps.push_back(nullptr);
        continue;
      }

      // Transform to world coordinate
      pt_3d = T_1_w.inv() * pt_3d;

      // Create a new map point
      vslam_datastructure::MapPoint::SharedPtr mp = std::make_shared<vslam_datastructure::MapPoint>();
      mp->set_pos(cv::Point3d(pt_3d.at<double>(0, 0), pt_3d.at<double>(1, 0), pt_3d.at<double>(2, 0)));

      new_mps.push_back(mp);
    }

    return new_mps;
  }

}  // namespace vslam_mapper_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_mapper_plugins::OpenCVMapper, vslam_mapper::base::Mapper)