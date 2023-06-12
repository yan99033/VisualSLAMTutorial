#include "vslam_utils/undistorter.hpp"

namespace {
  inline double MatRowMul(const cv::Mat& m, const double x, const double y, const double z, const int r) {
    return m.at<double>(r, 0) * x + m.at<double>(r, 1) * y + m.at<double>(r, 2) * z;
  }

  void initUndistortRectifyMap(const cv::Mat& K, const cv::Mat& D, const double xid, const cv::Mat& R, const cv::Mat& P,
                               const cv::Size& size, cv::Mat& map1, cv::Mat& map2) {
    map1 = cv::Mat(size, CV_32F);
    map2 = cv::Mat(size, CV_32F);

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    double s = K.at<double>(0, 1);

    // double xid = xi.at<double>(0, 0);

    double k1 = D.at<double>(0, 0);
    double k2 = D.at<double>(0, 1);
    double p1 = D.at<double>(0, 2);
    double p2 = D.at<double>(0, 3);

    cv::Mat KRi = (P * R).inv();

    for (int r = 0; r < size.height; ++r) {
      for (int c = 0; c < size.width; ++c) {
        double xc = MatRowMul(KRi, c, r, 1., 0);
        double yc = MatRowMul(KRi, c, r, 1., 1);
        double zc = MatRowMul(KRi, c, r, 1., 2);

        double rr = sqrt(xc * xc + yc * yc + zc * zc);
        double xs = xc / rr;
        double ys = yc / rr;
        double zs = zc / rr;

        double xu = xs / (zs + xid);
        double yu = ys / (zs + xid);

        double r2 = xu * xu + yu * yu;
        double r4 = r2 * r2;
        double xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu + p2 * (r2 + 2 * xu * xu);
        double yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu + p1 * (r2 + 2 * yu * yu);

        double u = fx * xd + s * yd + cx;
        double v = fy * yd + cy;

        map1.at<float>(r, c) = static_cast<float>(u);
        map2.at<float>(r, c) = static_cast<float>(v);
      }
    }
  }
}  // namespace

namespace vslam_utils {
  namespace camera {
    Undistorter::Undistorter(const cv::Mat& K, const int image_width, const int image_height,
                             const cv::Mat& dist_coeffs)
        : K_{K}, image_width_{image_width}, image_height_{image_height}, dist_coeffs_{dist_coeffs} {
      if (dist_coeffs.empty() || cv::sum(dist_coeffs) == cv::Scalar(0.0)) {
        return;
      }

      passthrough_ = false;

      K_ = cv::getOptimalNewCameraMatrix(K, dist_coeffs, cv::Size(image_width, image_height), 0,
                                         cv::Size(image_width, image_height), nullptr, false);

      cv::initUndistortRectifyMap(K, dist_coeffs, cv::Mat(), K_, cv::Size(image_width, image_height), CV_16SC2, map1_,
                                  map2_);
    }

    cv::Mat Undistorter::K() const { return K_.clone(); }

    cv::Mat Undistorter::undistortImage(const cv::Mat& in_image) const {
      if (passthrough_) {
        return in_image;
      }

      cv::Mat out_image;
      cv::remap(in_image, out_image, map1_, map2_, cv::INTER_LINEAR);

      return out_image;
    }

    CalicamUndistorter::CalicamUndistorter(const cv::Mat& K, const int image_width, const int image_height,
                                           const cv::Mat& dist_coeffs, const double xi, const cv::Mat& R,
                                           const cv::Mat& K_new)
        : Undistorter(K_new, image_width, image_height), K_ori_{K}, xi_{xi}, R_{R} {
      passthrough_ = false;
      dist_coeffs_ = dist_coeffs;
      calculateUndistortRectifyMap();
    }

    void CalicamUndistorter::calculateUndistortRectifyMap() {
      initUndistortRectifyMap(K_ori_, dist_coeffs_, xi_, R_, K_, cv::Size(image_width_, image_height_), map1_, map2_);
    }
  }  // namespace camera
}  // namespace vslam_utils