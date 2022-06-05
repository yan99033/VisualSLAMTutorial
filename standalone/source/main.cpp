#include <cxxopts.hpp>
#include <iostream>
#include <string>

#include "image_loader/load_from_folder.h"

auto main(int argc, char** argv) -> int {
  cxxopts::Options options(*argv, "Visual SLAM on a images in a folder");

  std::string language;
  std::string name;

  // clang-format off
  options.add_options()
    ("h,help", "Show help")
    ("v,version", "1.0")
  ;
  // clang-format on

  auto result = options.parse(argc, argv);

  if (result["help"].as<bool>()) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  vslam_libs::image_loader::LoadFromFolder loader(
      "/media/bryan/DATA/EuRoC_dataset/MH/MH_01/cam0/data");

  return 0;
}
