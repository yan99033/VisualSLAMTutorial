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

    void Visualizer::addCurrentFrame(vslam_libs::datastructure::FramePtr frame) {
      std::lock_guard<std::mutex> lck(mutex);
      curr_frame = frame;
      drawn_curr_frame = false;
    }

    void Visualizer::stop() {
      running = false;
      viewer_thread.join();
    }

    void Visualizer::drawFrame(vslam_libs::datastructure::Frame* frame) {
      // MatrixCFf cam_marker;
      // cam_marker << 0.0, -0.1, 0.0, -0.1, 0.0, 0.1, 0.0, 0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1,
      // 0.1,
      //     -0.1, 0.0, -0.06, 0.0, 0.06, 0.0, -0.06, 0.0, 0.06, -0.06, 0.06, 0.06, 0.06, 0.06,
      //     -0.06, -0.06, -0.06, 0.0, 0.06, 0.0, 0.06, 0.0, 0.06, 0.0, 0.06, 0.06, 0.06, 0.06,
      //     0.06, 0.06, 0.06, 0.06,
      //     0.06, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

      // Sophus::SE3d Tcw;
      // frame->getPose(Tcw);
      // cam_marker = Tcw.matrix().cast<float>() * cam_marker * 10;

      // glColor3f(0, 1.0, 0);
      // glLineWidth(3);
      // glBegin(GL_LINES);
      // for (int i = 0; i < 16; i++) {
      //   glVertex3f(cam_marker.coeff(0, i), cam_marker.coeff(1, i), cam_marker.coeff(2, i));
      //   std::cout << "Point: " << cam_marker.coeff(0, i) << " " << cam_marker.coeff(1, i) << " "
      //             << cam_marker.coeff(2, i) << std::endl;
      // }

      // glEnd();

      Sophus::SE3d Tcw;
      frame->getPose(Tcw);

      std::cout << "global pose2:"
                << "\n";
      std::cout << Tcw.rotationMatrix() << "\n";
      std::cout << Tcw.translation() << std::endl;

      const float sz = 1.0;
      const int line_width = 2.0;
      const float fx = 400;
      const float fy = 400;
      const float cx = 512;
      const float cy = 384;
      const float width = 1080;
      const float height = 768;

      glPushMatrix();

      Sophus::Matrix4f m = Tcw.matrix().cast<float>();
      glMultMatrixf((GLfloat*)m.data());

      glColor3f(0.0, 1.0, 0.0);

      glLineWidth(line_width);
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(0, 0, 0);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

      glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

      glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

      glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
      glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

      glEnd();
      glPopMatrix();
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
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        d_cam.Activate(s_cam);

        // Draw the current frame
        if (curr_frame.get()) {
          drawFrame(curr_frame.get());
          drawn_curr_frame = true;
        }

        // Swap frames and Process Events
        pangolin::FinishFrame();
      }
    }

  }  // namespace visualizer
}  // namespace vslam_libs