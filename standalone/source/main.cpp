#include <fmt/core.h>

#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "camera_tracker/camera_tracker.h"
#include "datastructure/frame.h"
#include "feature_detector/feature_detector.h"
#include "image_loader/load_from_folder.h"
#include "visualizer/visualizer.h"

// Must come after the local library
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>

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

  // Visualizer
  vslam_libs::visualizer::Visualizer visualizer;

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
    vslam_libs::feature_detector::OrbFeatureDetector detector(num_features);
    auto matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

    // Camera tracker
    vslam_libs::camera_tracker::CameraTracker cam_tracker(cam_mat, matcher.get());

    std::vector<vslam_libs::datastructure::FramePtr> frames;
    vslam_libs::datastructure::FramePtr prev_frame;

    Sophus::SE3d Tpw;
    cv::Mat Rcp_cv, tcp_cv, mask;
    Eigen::Matrix3d Rcp_eigen;
    Eigen::Vector3d tcp_eigen;

    while (true) {
      cv::Mat image = loader.getNextFrame();

      vslam_libs::datastructure::FramePtr frame
          = std::make_shared<vslam_libs::datastructure::Frame>(image, cam_mat, &detector);

      frames.push_back(frame);

      if (frames.size() < 2) {
        prev_frame = frame;
        continue;
      }

      // recovering the pose and the essential matrix
      cam_tracker.recoverPose(frame, prev_frame, Rcp_cv, tcp_cv, mask);

      cv::cv2eigen(Rcp_cv, Rcp_eigen);
      cv::cv2eigen(tcp_cv, tcp_eigen);
      Sophus::SE3d Tcp(Rcp_eigen, tcp_eigen);

      // Triangulate point

      // Add global 3D points to the frame

      // Visualize the 3D points of the current frame

      // // Calculate the global pose
      prev_frame->getPose(Tpw);
      Sophus::SE3d Tcw = Tcp * Tpw;
      frame->setPose(Tcw);

      std::cout << "global pose:"
                << "\n";
      std::cout << Tcw.rotationMatrix() << "\n";
      std::cout << Tcw.translation() << std::endl;

      cv::imshow("image", image);
      cv::waitKey(1);

      visualizer.addCurrentFrame(frame);

      // Keep the pointer to the previous frame
      prev_frame = frame;
    }
  } catch (const std::exception& e) {
    fmt::print("Error: {}\n", e.what());
  } catch (...) {
    fmt::print("Non std::exception error caught\n");
  }

  visualizer.stop();

  fmt::print("Goodbye!\n");

  return 0;
}
