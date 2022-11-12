#include "visualizer/visualizer.h"

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/scene/scenehandler.h>

namespace vslam_libs {
  namespace visualizer {
    Visualizer::Visualizer() {
      running = true;
      viewer_thread = std::thread(&Visualizer::threadLoop, this);
    }

    void Visualizer::stop() {
      running = false;
      viewer_thread.join();
    }

    void Visualizer::threadLoop() {
      // create a window and bind its context to the main thread
      pangolin::CreateWindowAndBind(window_name, 640, 480);

      // enable depth
      glEnable(GL_DEPTH_TEST);

      // Define Projection and initial ModelView matrix
      pangolin::OpenGlRenderState s_cam(
          pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
          pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

      // Create Interactive View in window
      pangolin::Handler3D handler(s_cam);
      pangolin::View& d_cam = pangolin::CreateDisplay()
                                  .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                                  .SetHandler(&handler);

      while (!pangolin::ShouldQuit() and running) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
      }
    }

  }  // namespace visualizer
}  // namespace vslam_libs