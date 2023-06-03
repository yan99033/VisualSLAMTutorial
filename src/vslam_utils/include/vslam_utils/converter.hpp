// TODO: Licence

#ifndef VSLAM_UTILS__CONVERTER_HPP_
#define VSLAM_UTILS__CONVERTER_HPP_

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

namespace vslam_utils {

  namespace conversions {
    inline Eigen::Vector2d cvPoint2fToEigenVector2d(const cv::Point2f& pt) { return Eigen::Vector2d(pt.x, pt.y); }

    inline Eigen::Vector3d cvPoint3dToEigenVector3d(const cv::Point3d& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); }

    inline cv::Mat eigenRotationTranslationToCvMat(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
      return (cv::Mat_<double>(4, 4) << R.coeff(0, 0), R.coeff(0, 1), R.coeff(0, 2), t.x(), R.coeff(1, 0),
              R.coeff(1, 1), R.coeff(1, 2), t.y(), R.coeff(2, 0), R.coeff(2, 1), R.coeff(2, 2), t.z(), 0.0, 0.0, 0.0,
              1.0);
    }

    inline cv::Point3d eigenVector3dToCvPoint3d(const Eigen::Vector3d& vec) {
      return cv::Point3d(vec.x(), vec.y(), vec.z());
    }

    inline int encoding2mat_type(const std::string& encoding) {
      if (encoding == "mono8") {
        return CV_8UC1;
      } else if (encoding == "bgr8") {
        return CV_8UC3;
      } else if (encoding == "mono16") {
        return CV_16SC1;
      } else if (encoding == "rgba8") {
        return CV_8UC4;
      }
      throw std::runtime_error("Unsupported mat type");
    }

    inline std::string mat_type2encoding(int mat_type) {
      switch (mat_type) {
        case CV_8UC1:
          return "mono8";
        case CV_8UC3:
          return "bgr8";
        case CV_16SC1:
          return "mono16";
        case CV_8UC4:
          return "rgba8";
        default:
          throw std::runtime_error("Unsupported encoding type");
      }
    }

    cv::Mat to_transformation_matrix(const cv::Mat& R, const cv::Mat& t);

  }  // namespace conversions

}  // namespace vslam_utils

#endif  // VSLAM_UTILS__CONVERTER_HPP_