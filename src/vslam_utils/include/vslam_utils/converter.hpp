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

    cv::Mat to_transformation_matrix(const cv::Mat& R, const cv::Mat& t);

  }  // namespace conversions

}  // namespace vslam_utils

#endif  // VSLAM_UTILS__CONVERTER_HPP_