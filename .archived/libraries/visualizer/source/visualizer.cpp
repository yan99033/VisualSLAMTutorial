#include "visualizer/visualizer.h"

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

namespace vslam_libs {
  namespace visualizer {
    Visualizer::Visualizer() {
      running = true;
      viewer_thread = std::thread(&Visualizer::threadLoop, this);
    }

    void Visualizer::addCurrentFrame(vslam_libs::datastructure::FramePtr frame) {
      std::lock_guard<std::mutex> lck(mutex);
      curr_frame = frame;
      drawn_curr_frame = false;

      all_frames.push_back(frame);
    }

    void Visualizer::stop() {
      running = false;
      viewer_thread.join();
    }

    void Visualizer::followCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
      Sophus::SE3d Twc;
      curr_frame->getPose(Twc);
      // Sophus::SE3d Twc = Tcw.inverse();

      pangolin::OpenGlMatrix m(Twc.matrix());
      vis_camera.Follow(m, true);
    }

    void Visualizer::drawFrame(vslam_libs::datastructure::Frame* frame) {
      Sophus::SE3d Tcw;
      frame->getPose(Tcw);
      Sophus::SE3d Twc = Tcw.inverse();

      const int line_width = 2.0f;
      const float w = 0.5f;
      const float h = w * 0.75;
      const float z = w * 0.6;

      glPushMatrix();

      Sophus::Matrix4f m = Twc.matrix().cast<float>();

      glMultMatrixf((GLfloat*)m.data());

      glLineWidth(line_width);
      glColor3f(0.0, 0.0, 1.0);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);

      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);

      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);

      glEnd();
      glPopMatrix();
    }

    void Visualizer::drawFrames() {
      std::lock_guard<std::mutex> lck(mutex);
      for (const auto& frame : all_frames) {
        drawFrame(frame.get());
      }
    }

    void Visualizer::threadLoop() {
      // create a window and bind its context to the main thread
      pangolin::CreateWindowAndBind(window_name, 1024, 768);

      // enable depth
      glEnable(GL_DEPTH_TEST);

      // Define Projection and initial ModelView matrix
      pangolin::OpenGlRenderState s_cam(
          pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
          pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

      // Create Interactive View in window
      pangolin::Handler3D handler(s_cam);
      pangolin::View& d_cam
          = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(&handler);

      while (!pangolin::ShouldQuit() and running) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // Draw the current frame
        if (curr_frame.get()) {
          drawFrame(curr_frame.get());
          drawn_curr_frame = true;

          // followCurrentFrame(s_cam);
        }

        drawFrames();

        // Swap frames and Process Events
        pangolin::FinishFrame();
      }
    }

  }  // namespace visualizer
}  // namespace vslam_libs