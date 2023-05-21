// TODO: Licence

#ifndef VSLAM_UTILS__UNDISTORTER_HPP_
#define VSLAM_UTILS__UNDISTORTER_HPP_

#include <opencv2/opencv.hpp>

namespace vslam_utils {

  namespace abstract {
    class Undistorter {
    public:
      virtual ~Undistorter() {}

      virtual cv::Mat get_K() const = 0;

      virtual cv::Mat undistort_image(const cv::Mat& in_image) const = 0;
    };

    class CalicamUndistorter : public virtual Undistorter {
    private:
      virtual void calculate_undistort_rectify_map() = 0;
    };
  }  // namespace abstract

  class Undistorter : public virtual abstract::Undistorter {
  public:
    // Given the camera matrix and distortion coefficients,
    // Undistorter calculates the new camera matrix and undistorted image
    Undistorter(const cv::Mat& K, const int image_width, const int image_height,
                const cv::Mat& dist_coeffs = cv::Mat());

    cv::Mat get_K() const override;

    cv::Mat undistort_image(const cv::Mat& in_image) const override;

  protected:
    // If no distortion coefficient is provided, we just pass the image and camera matrix through
    bool passthrough_{true};

    cv::Mat K_;

    int image_width_;
    int image_height_;

    cv::Mat dist_coeffs_;

    cv::Mat map1_;
    cv::Mat map2_;
  };

  class CalicamUndistorter : public virtual abstract::CalicamUndistorter, public Undistorter {
  public:
    CalicamUndistorter(const cv::Mat& K, const int image_width, const int image_height, const cv::Mat& dist_coeffs,
                       const double xi, const cv::Mat& R, const cv::Mat& K_new);

  private:
    void calculate_undistort_rectify_map() override;

    cv::Mat K_ori;

    double xi;
    cv::Mat R;
  };

}  // namespace vslam_utils

#endif  // VSLAM_UTILS__UNDISTORTER_HPP_