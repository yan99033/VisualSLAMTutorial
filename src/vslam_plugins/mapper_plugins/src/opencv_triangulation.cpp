#include "vslam_mapper_plugins/opencv_triangulation.hpp"

#include <cmath>
#include <iostream>

namespace vslam_mapper_plugins {
  void OpenCvTriangulation::initialize(const cv::Mat& K) { this->K = K; }

  void OpenCvTriangulation::map(vslam_datastructure::MatchedPoints& matched_points) {
    // Get the matched points that don't have a map point

    // Triangulate the map points

    // Create map points for the points
  }

}  // namespace vslam_mapper_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_mapper_plugins::OpenCvTriangulation, vslam_mapper_base::Mapper)