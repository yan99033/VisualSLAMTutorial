#include "image_loader/load_from_folder.h"

#include <fmt/core.h>

#include <algorithm>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
namespace vslam_libs {
  namespace image_loader {

    LoadFromFolder::LoadFromFolder(const std::string& folder, std::string ext) : i(0) {
      // Check if the path points to an empty file or directory
      if (!std::filesystem::exists(folder) or std::filesystem::is_empty(folder)) {
        throw ImageLoaderError(
            fmt::format("The folder ({}) is either invalid or empty.\n", folder));
      }

      for (const auto& f : std::filesystem::directory_iterator(folder)) {
        if (f.path().extension() == ext) {
          files.push_back(f.path());
        }
      }

      std::sort(files.begin(), files.end());

      if (files.empty()) {
        throw ImageLoaderError(fmt::format("The folder({}) does not have images\n", folder));
      }

      fmt::print("Loaded {} files\n", files.size());
    }

    cv::Mat LoadFromFolder::getNextFrame() {
      cv::Mat frame = cv::imread(files.at(i), cv::IMREAD_COLOR);

      // Keep replaying the last frame if it reaches the end of the sequence
      if (i < files.size() - 1) {
        i += 1;
      }

      return frame;
    }

  }  // namespace image_loader
}  // namespace vslam_libs
