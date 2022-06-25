#include <fmt/core.h>

#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "datastructure/frame.h"
#include "feature_detector/feature_detector.h"
#include "image_loader/load_from_folder.h"

void getCorrespondences(const std::vector<cv::KeyPoint>& keypoints1,
                        const std::vector<cv::KeyPoint>& keypoints2,
                        const std::vector<cv::DMatch>& matches, const std::vector<char>& mask,
                        std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2) {
  // Get the matches
  points1.clear();
  points2.clear();

  // Check if the length of matches is the same as that of mask!
  for (size_t m = 0; m < matches.size(); m++) {
    if (mask.empty() || mask[m]) {
      int i1 = matches[m].queryIdx;
      int i2 = matches[m].trainIdx;

      cv::KeyPoint kp1 = keypoints1.at(i1);
      cv::KeyPoint kp2 = keypoints2.at(i2);

      points1.emplace_back(kp1.pt);
      points2.emplace_back(kp2.pt);
    }
  }
}

auto main(int argc, char** argv) -> int {
  cxxopts::Options options(*argv, "Visual SLAM on a images in a folder");

  std::string language;
  std::string name;

  // clang-format off
  options.add_options()
    ("h,help", "Show help")
    ("v,version", "1.0")
    ("image_folder", "A folder containing the images", cxxopts::value<std::string>())
    ("intrinsics", "camera intrinsics in a yaml file", cxxopts::value<std::string>()->default_value("standalone/config/cam.json"))
  ;
  // clang-format on

  auto result = options.parse(argc, argv);

  if (result["help"].as<bool>()) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  try {
    // Load camera intrinsics
    double fx, fy, cx, cy;
    std::string intrinsics_yaml = result["intrinsics"].as<std::string>();
    std::ifstream i(intrinsics_yaml);
    nlohmann::json intrinsics;
    i >> intrinsics;
    intrinsics.at("fx").get_to(fx);
    intrinsics.at("fy").get_to(fy);
    intrinsics.at("cx").get_to(cx);
    intrinsics.at("cy").get_to(cy);
    fmt::print("intrinsics: {} {} {} {}\n", fx, fy, cx, cy);
    cv::Mat cam_mat = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Get the image folder
    std::string image_folder = result["image_folder"].as<std::string>();

    vslam_libs::image_loader::LoadFromFolder loader(image_folder);

    // ORB feature detector and matcher
    int num_features = 1000;
    cv::Ptr<cv::ORB> detector2 = cv::ORB::create(num_features);
    vslam_libs::feature_detector::OrbFeatureDetector detector(num_features);
    auto matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

    bool first_frame = true;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    while (true) {
      cv::Mat image = loader.getNextFrame();

      if (first_frame) {
        first_frame = false;
        detector2->detectAndCompute(image, cv::noArray(), keypoints1, descriptors1);
      }

      vslam_libs::datastructure::FramePtr frame
          = std::make_shared<vslam_libs::datastructure::Frame>(image, cam_mat, &detector);

      detector2->detectAndCompute(image, cv::noArray(), keypoints2, descriptors2);

      std::vector<cv::DMatch> matches;
      std::vector<char> matcher_mask;
      matcher->match(descriptors1, descriptors2, matches, matcher_mask);

      std::vector<cv::Point2f> points1, points2;
      getCorrespondences(keypoints1, keypoints2, matches, matcher_mask, points1, points2);

      // recovering the pose and the essential matrix
      cv::Mat E, R, t, mask;
      E = cv::findEssentialMat(points2, points1, cam_mat, cv::RANSAC, 0.999, 1.0, mask);
      cv::recoverPose(E, points2, points1, cam_mat, R, t, mask);

      std::cout << "pose:\n";
      std::cout << R << "\n";
      std::cout << t << std::endl;
      cv::imshow("image", image);
      cv::waitKey(1);

      keypoints1 = keypoints2;
      descriptors1 = descriptors2;
    }
  } catch (const std::exception& e) {
    fmt::print("Error: {}\n", e.what());
  } catch (...) {
    fmt::print("Non std::exception error caught\n");
  }

  fmt::print("Goodbye!\n");

  return 0;
}
