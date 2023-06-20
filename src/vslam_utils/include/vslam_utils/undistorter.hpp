/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VSLAM_UTILS__UNDISTORTER_HPP_
#define VSLAM_UTILS__UNDISTORTER_HPP_

#include <opencv2/opencv.hpp>

namespace vslam_utils {
  namespace camera {
    namespace abstract {
      class Undistorter {
      public:
        /// Destructor
        virtual ~Undistorter() {}

        /// Camera matrix
        /**
         * [[fx,  0, cx],
         *  [ 0, fy, cy],
         *  [ 0,  0,  1]]
         * \return 3x3 camera matrix
         */
        virtual cv::Mat K() const = 0;

        /// Undistort image
        /**
         * \return undistorted image
         */
        virtual cv::Mat undistortImage(const cv::Mat& in_image) const = 0;
      };

      class CalicamUndistorter : public virtual Undistorter {
      private:
        /// Calculate undistort rectify map for image undistortion
        virtual void calculateUndistortRectifyMap() = 0;
      };
    }  // namespace abstract

    class Undistorter : public virtual abstract::Undistorter {
    public:
      /// Given the camera matrix and distortion coefficients, Undistorter calculates the new camera matrix and
      /// undistorted image
      /**
       * \param[in] K camera matrix
       * \param[in] image_width image width
       * \param[in] image_height image height
       * \param[in] dist_coeffs distortion coefficients
       */
      Undistorter(const cv::Mat& K, const int image_width, const int image_height,
                  const cv::Mat& dist_coeffs = cv::Mat());

      /// Get camera matrix
      /**
       * \return 3x3 camera matrix
       */
      cv::Mat K() const override;

      /// Undistort image
      /**
       * \return undistorted image
       */
      cv::Mat undistortImage(const cv::Mat& in_image) const override;

    protected:
      /// If enabled, no undistortion and re-calculation of a new camera matrix are performed
      bool passthrough_{true};

      /// Camera matrix
      cv::Mat K_;

      /// Image width
      int image_width_;

      /// Image height
      int image_height_;

      /// Distortion coefficient
      cv::Mat dist_coeffs_;

      /// Undistortion and rectification transformation map 1
      /**
       * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
       */
      cv::Mat map1_;

      /// Undistortion and rectification transformation map 2
      /**
       * \sa https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
       */
      cv::Mat map2_;
    };

    class CalicamUndistorter : public virtual abstract::CalicamUndistorter, public Undistorter {
    public:
      /// CalicamUndistorter constructor
      // https://github.com/astar-ai/calicam
      /// \note I haven't done extensive tests with the camera yet
      /**
       * \param[in] K camera matrix
       * \param[in] image_width image width
       * \param[in] image_height image height
       * \param[in] dist_coeffs distortion coefficients
       * \param[in] xi parameter used in calculating undistortion and rectification transformation map
       * \param[in] R parameter used in calculating undistortion and rectification transformation map
       * \param[in] K_new new camera matrix
       */
      CalicamUndistorter(const cv::Mat& K, const int image_width, const int image_height, const cv::Mat& dist_coeffs,
                         const double xi, const cv::Mat& R, const cv::Mat& K_new);

    private:
      /// Calculate undistortion and rectification transformation map
      void calculateUndistortRectifyMap() override;

      /// The original camera matrix
      cv::Mat K_ori_;

      /// Parameter used in calculating undistortion and rectification transformation map
      double xi_;

      /// Parameter used in calculating undistortion and rectification transformation map
      cv::Mat R_;
    };
  }  // namespace camera
}  // namespace vslam_utils

#endif  // VSLAM_UTILS__UNDISTORTER_HPP_