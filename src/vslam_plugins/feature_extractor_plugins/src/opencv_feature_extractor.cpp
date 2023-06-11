#include "vslam_feature_extractor_plugins/opencv_feature_extractor.hpp"

namespace {
  using CvMatPyr = std::vector<cv::Mat>;

  // Function to calculate ORB features (WTA_K = 2).
  // Credit:
  // https://github.com/opencv/opencv/blob/f5a92cb43f6ac6b60f401613cc80cea3a04cf59b/modules/features2d/src/orb.cpp

  bool calculateICAngle(const cv::Mat& img, cv::KeyPoint& kp, const double scale_factor, const std::vector<int>& u_max,
                        int half_k) {
    if ((cvRound(kp.pt.x * scale_factor - kp.size) <= 0) || (cvRound(kp.pt.x * scale_factor + kp.size) >= img.cols)
        || (cvRound(kp.pt.y * scale_factor - kp.size) <= 0)
        || (cvRound(kp.pt.y * scale_factor + kp.size) >= img.rows)) {
      return false;
    }

    int step = static_cast<int>(img.step1());
    int m_01 = 0;
    int m_10 = 0;
    const uchar* center = &img.at<uchar>(cvRound(kp.pt.y * scale_factor), cvRound(kp.pt.x * scale_factor));

    // Treat the center line differently, v=0
    for (int u = -half_k; u <= half_k; ++u) m_10 += u * center[u];

    // Go line by line in the circular patch
    for (int v = 1; v <= half_k; ++v) {
      // Proceed over the two lines
      int v_sum = 0;
      int d = u_max[v];
      for (int u = -d; u <= d; ++u) {
        int val_plus = center[u + v * step];
        int val_minus = center[u - v * step];
        v_sum += (val_plus - val_minus);
        m_10 += u * (val_plus + val_minus);
      }
      m_01 += v * v_sum;
    }

    kp.angle = cv::fastAtan2((float)m_01, (float)m_10);

    return true;
  }

  bool calculateHarrisResponseAndOctave(const CvMatPyr image_pyr, cv::KeyPoint& kp, const size_t nlevels,
                                        const std::vector<double>& scale_factors, const int patch_size,
                                        const int harris_block_size, const float harris_k = 0.04) {
    assert(!image_pyr.empty() && (image_pyr.size() == nlevels) && (scale_factors.size() == nlevels));

    // Set the initial response to zero so that we can find a higher response later
    kp.response = 0;

    int r = harris_block_size / 2;

    float scale = 1.f / ((1 << 2) * harris_block_size * 255.f);
    float scale_sq_sq = scale * scale * scale * scale;

    bool is_good{false};

    for (size_t octave = 0; octave < nlevels; octave++) {
      cv::Mat img = image_pyr.at(octave);
      CV_CheckTypeEQ(img.type(), CV_8UC1, "");
      const uchar* ptr00 = img.ptr<uchar>();
      size_t size_t_step = img.step;
      CV_CheckLE(size_t_step * harris_block_size + harris_block_size + 1, (size_t)INT_MAX,
                 "");  // ofs computation, step+1
      int step = static_cast<int>(size_t_step);

      cv::AutoBuffer<int> ofsbuf(harris_block_size * harris_block_size);
      int* ofs = ofsbuf.data();
      for (int i = 0; i < harris_block_size; i++)
        for (int j = 0; j < harris_block_size; j++) ofs[i * harris_block_size + j] = (int)(i * step + j);

      int x0 = cvRound(kp.pt.x * scale_factors[octave]);
      int y0 = cvRound(kp.pt.y * scale_factors[octave]);

      if ((x0 - 1 <= r) || (x0 + r + 1 >= img.cols) || (y0 - 1 <= r) || (y0 + r + 1 >= img.rows)) {
        continue;
      }

      const uchar* ptr0 = ptr00 + (y0 - r) * size_t_step + (x0 - r);
      int a = 0;
      int b = 0;
      int c = 0;

      for (int k = 0; k < harris_block_size * harris_block_size; k++) {
        const uchar* ptr = ptr0 + ofs[k];
        int Ix = (ptr[1] - ptr[-1]) * 2 + (ptr[-step + 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[step - 1]);
        int Iy = (ptr[step] - ptr[-step]) * 2 + (ptr[step - 1] - ptr[-step - 1]) + (ptr[step + 1] - ptr[-step + 1]);
        a += Ix * Ix;
        b += Iy * Iy;
        c += Ix * Iy;
      }

      float response = ((float)a * b - (float)c * c - harris_k * ((float)a + b) * ((float)a + b)) * scale_sq_sq;
      if (response > kp.response) {
        kp.response = response;
        kp.octave = octave;
        kp.size = static_cast<float>(patch_size) / scale_factors.at(octave);

        is_good = true;
      }
    }
    return is_good;
  }
}  // namespace

namespace vslam_feature_extractor_plugins {
  void OpenCVFeatureExtractor::initialize(int num_features, const vslam_datastructure::Point::Type point_type) {
    num_features_ = num_features;
    point_type_ = point_type;

    switch (point_type) {
      case vslam_datastructure::Point::Type::orb:
        feature_detector_ = cv::ORB::create(num_features);
        break;
      default:
        throw std::runtime_error("OpenCVFeatureExtractor::initialize: feature detector is not defined.");
    }
  }

  vslam_datastructure::Points OpenCVFeatureExtractor::extractFeatures(const cv::Mat& image) {
    // Convert the image to greyscale
    cv::Mat grey_image;
    cv::cvtColor(image, grey_image, cv::COLOR_BGR2GRAY);

    auto keypoints = calculateKeypoints(grey_image);

    cv::Mat descriptors;
    feature_detector_->compute(grey_image, keypoints, descriptors);

    vslam_datastructure::Points ft_points;
    for (size_t i = 0; i < keypoints.size(); i++) {
      auto pt = std::make_shared<vslam_datastructure::Point>(keypoints[i], descriptors.row(i).clone(), point_type_);
      ft_points.push_back(pt);
    }
    return ft_points;
  }

  OpenCVFeatureExtractor::KeyPoints OpenCVFeatureExtractor::calculateKeypoints(const cv::Mat& grey_image) {
    std::vector<cv::Point2d> corners;
    cv::goodFeaturesToTrack(grey_image, corners, num_features_, quality_level_, min_dist_);

    // Calculate the pyramid info
    CvMatPyr image_pyramid;
    std::vector<double> scale_factors;
    double curr_scale_factor = 1.0;
    for (size_t i = 0; i < nlevels_; i++) {
      scale_factors.push_back(curr_scale_factor);
      curr_scale_factor /= scale_factor_;
      if (i == 0) {
        image_pyramid.push_back(grey_image);

        continue;
      }

      int width_pyr = cvRound(grey_image.cols * scale_factors.back());
      int height_pyr = cvRound(grey_image.rows * scale_factors.back());
      cv::Mat resized;
      cv::resize(image_pyramid.back(), resized, cv::Size(width_pyr, height_pyr));
      image_pyramid.push_back(resized);
    }

    // pre-compute the end of a row in a circular patch
    int half_patch_size = patch_size_ / 2;
    std::vector<int> umax(half_patch_size + 2);

    int v, v0, vmax = cvFloor(half_patch_size * std::sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(half_patch_size * std::sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v) umax[v] = cvRound(std::sqrt((double)half_patch_size * half_patch_size - v * v));

    // Make sure we are symmetric
    for (v = half_patch_size, v0 = 0; v >= vmin; --v) {
      while (umax[v0] == umax[v0 + 1]) ++v0;
      umax[v] = v0;
      ++v0;
    }

    KeyPoints keypoints;
    keypoints.reserve(corners.size());
    for (const auto& corner : corners) {
      cv::KeyPoint kp(corner, 1.0f);
      if (calculateHarrisResponseAndOctave(image_pyramid, kp, nlevels_, scale_factors, patch_size_, harris_block_size_)
          && calculateICAngle(image_pyramid.at(kp.octave), kp, scale_factors.at(kp.octave), umax, half_patch_size)) {
        keypoints.push_back(kp);
      }
    }

    return keypoints;
  }
}  // namespace vslam_feature_extractor_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_feature_extractor_plugins::OpenCVFeatureExtractor,
                       vslam_feature_extractor::base::FeatureExtractor)