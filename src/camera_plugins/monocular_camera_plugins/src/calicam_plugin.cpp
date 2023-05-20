#include "monocular_camera_plugins/calicam_plugin.hpp"

namespace {
  inline double MatRowMul(const cv::Mat& m, const double x, const double y, const double z, const int r) {
    return m.at<double>(r, 0) * x + m.at<double>(r, 1) * y + m.at<double>(r, 2) * z;
  }

  void initUndistortRectifyMap(const cv::Mat& K, const cv::Mat& D, const cv::Mat& xi, const cv::Mat& R,
                               const cv::Mat& P, const cv::Size& size, cv::Mat& map1, cv::Mat& map2) {
    map1 = cv::Mat(size, CV_64F);
    map2 = cv::Mat(size, CV_64F);

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    double s = K.at<double>(0, 1);

    double xid = xi.at<double>(0, 0);

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

        map1.at<double>(r, c) = u;
        map2.at<double>(r, c) = v;
      }
    }
  }
}  // namespace

namespace monocular_camera_plugins {
  CaliCam::~CaliCam() { video_capture_.release(); }

  void CaliCam::initialize(const std::string& params_file) {
    cv::FileStorage fs(params_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cout << "Failed to open ini parameters" << std::endl;
      throw std::runtime_error("failed to open " + params_file);
    }
    cv::Mat K;
    cv::Mat dist_coeffs;
    cv::Mat xi;
    cv::Mat R;
    double fov;

    fs["kl"] >> K;
    fs["Dl"] >> dist_coeffs;
    fs["fov"] >> fov;
    fs["xil"] >> xi;
    fs["Rl"] >> R;
    fs["image_height"] >> image_height_;
    fs["image_width"] >> image_width_;
    fs["camera_id"] >> camera_id_;

    fs.release();

    if (image_height_ <= 0) {
      throw std::runtime_error("invalid image height (" + std::to_string(image_height_) + ")");
    }

    if (image_width_ <= 0) {
      throw std::runtime_error("invalid image width (" + std::to_string(image_width_) + ")");
    }

    // calculate the initial maps for undistortion
    double fov_rad = fov * CV_PI / 180.;
    double focal = image_height_ / 2. / tan(fov_rad / 2.);
    cv::Mat K_ = (cv::Mat_<double>(3, 3) << focal, 0., image_width_ / 2. - 0.5, 0., focal, image_height_ / 2. - 0.5, 0.,
                  0., 1.);
    cv::Size image_size(image_width_, image_height_);
    initUndistortRectifyMap(K, dist_coeffs, xi, R, K_, image_size, map1_, map2_);

    video_capture_.open(camera_id_);
    video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    if (!video_capture_.isOpened()) {
      throw std::runtime_error("Cannot open camera " + std::to_string(camera_id_));
    }
  }

  cv::Mat CaliCam::grab_image() {
    cv::Mat image;

    if (!video_capture_.read(image)) {
      image = last_image_;
    } else {
      last_image_ = image.clone();
    }

    return image;
  }

}  // namespace monocular_camera_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(monocular_camera_plugins::CaliCam, camera_plugins_base::MonocularCamera)