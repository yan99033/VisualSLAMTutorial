#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <atomic>
#include <string>
#include <thread>
namespace vslam_libs {
  namespace visualizer {
    static const std::string window_name = "vSLAM";

    class Visualizer {
    public:
      Visualizer();

      void stop();

    private:
      void threadLoop();

      std::thread viewer_thread;

      std::atomic_bool running{false};
    };

  }  // namespace visualizer
}  // namespace vslam_libs

#endif  // __VISUALIZER_H__