#include <fmt/core.h>

#include <cxxopts.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "datastructure/frame.h"
#include "image_loader/load_from_folder.h"

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

    while (true) {
      cv::Mat image = loader.getNextFrame();

      vslam_libs::datastructure::FramePtr frame
          = std::make_shared<vslam_libs::datastructure::Frame>(image, cam_mat);

      cv::imshow("image", image);
      cv::waitKey(1);
    }
  } catch (const std::exception& e) {
    fmt::print("Error: {}\n", e.what());
  } catch (...) {
    fmt::print("Non std::exception error caught\n");
  }

  fmt::print("Goodbye!\n");

  return 0;
}
