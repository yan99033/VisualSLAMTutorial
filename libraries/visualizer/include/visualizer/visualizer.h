#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <Eigen/Core>
#include <atomic>
#include <string>
#include <thread>

#include "datastructure/frame.h"

namespace vslam_libs {
  namespace visualizer {
    static const std::string window_name = "vSLAM";

    class Visualizer {
    public:
      Visualizer();

      void stop();

      void addCurrentFrame(vslam_libs::datastructure::FramePtr frame);

    private:
      using MatrixCFf = Eigen::Matrix<float, 4, 16>;

      void threadLoop();

      void drawFrame(vslam_libs::datastructure::Frame* frame);

      std::thread viewer_thread;

      std::atomic_bool running{false};

      vslam_libs::datastructure::FramePtr curr_frame;

      std::atomic_bool drawn_curr_frame{false};

      std::mutex mutex;
    };

  }  // namespace visualizer
}  // namespace vslam_libs

#endif  // __VISUALIZER_H__