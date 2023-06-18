/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VSLAM_UTILS__CONVERTER_HPP_
#define VSLAM_UTILS__CONVERTER_HPP_

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

namespace vslam_utils {

  namespace conversions {
    /// Convert cv::Point2f to Eigen::Vector2d
    /**
     * \param[in] pt point of type cv::Point2f
     * \return point of type Eigen::Vector2d
     */
    inline Eigen::Vector2d cvPoint2fToEigenVector2d(const cv::Point2f& pt) { return Eigen::Vector2d(pt.x, pt.y); }

    /// Convert cv::Point3d to Eigen::Vector3d
    /**
     * \param[in] pt point of type cv::Point3d
     * \return point of type Eigen::Vector3d
     */
    inline Eigen::Vector3d cvPoint3dToEigenVector3d(const cv::Point3d& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); }

    /// Convert Eigen rotation matrix and translation vector to 4x4 transformation of type cv::Mat
    /**
     * \param[in] R 3x3 rotation matrix
     * \param[in] t 3x1 translation vector
     * \return 4x4 transformation metrix
     */
    inline cv::Mat eigenRotationTranslationToCvMat(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
      return (cv::Mat_<double>(4, 4) << R.coeff(0, 0), R.coeff(0, 1), R.coeff(0, 2), t.x(), R.coeff(1, 0),
              R.coeff(1, 1), R.coeff(1, 2), t.y(), R.coeff(2, 0), R.coeff(2, 1), R.coeff(2, 2), t.z(), 0.0, 0.0, 0.0,
              1.0);
    }

    /// Convert Eigen::Vector3d to cv::Point3d
    /**
     * \param[in] vec point of type Eigen::Vector3d
     * \return point of type cv::Point3d
     */
    inline cv::Point3d eigenVector3dToCvPoint3d(const Eigen::Vector3d& vec) {
      return cv::Point3d(vec.x(), vec.y(), vec.z());
    }

    /// Convert the encoding type to OpenCV Mat type
    /**
     * \param[in] encoding encoding in string format
     * \return OpenCV Mat type
     */
    inline int encodingToCvMatType(const std::string& encoding) {
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

    /// Convert the OpenCV Mat type to string encoding
    /**
     * \param[in] mat_type OpenCV Mat type
     * \return encoding in string format
     */
    inline std::string cvMatTypeToEncoding(int mat_type) {
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

    /// Copy the rotation matrix and translation vector to a 4x4 transformation matrix and return it
    /**
     * \param[in] R 3x3 rotation matrix
     * \param[in] t 3x1 translation vector
     * \return 4x4 transformation matrix
     */
    cv::Mat toTransformationMatrix(const cv::Mat& R, const cv::Mat& t);

    /// Convert rotation matrix to rotation angle in the axis-angle representation
    /**
     * \param[in] R rotation matrix
     * \return magnitude of the rotation matrix by the angle of the axis-angle representation
     */
    double rotationMatrixToRotationAngle(const cv::Mat& R);

  }  // namespace conversions

}  // namespace vslam_utils

#endif  // VSLAM_UTILS__CONVERTER_HPP_