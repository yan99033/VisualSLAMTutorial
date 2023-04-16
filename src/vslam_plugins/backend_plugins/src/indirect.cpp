#include "vslam_backend_plugins/indirect.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// #if defined G2O_HAVE_CHOLMOD
// G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
// #else
// G2O_USE_OPTIMIZATION_LIBRARY(eigen);
// #endif

namespace {
  Eigen::Vector3d cvPoint3dToEigenVector3d(const cv::Point3d& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); }

  g2o::SE3Quat cvMatToSE3Quat(const cv::Mat& pose) {
    Eigen::Matrix3d R;
    R << pose.at<double>(0, 0), pose.at<double>(0, 1), pose.at<double>(0, 2), pose.at<double>(1, 0),
        pose.at<double>(1, 1), pose.at<double>(1, 2), pose.at<double>(2, 0), pose.at<double>(2, 1),
        pose.at<double>(2, 2);
    Eigen::Vector3d t(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

    return g2o::SE3Quat(R, t);
  }

  cv::Mat eigenRotationTranslationToCvMat(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
    Eigen::Affine3d transformation_matrix = Eigen::Affine3d::Identity();
    transformation_matrix.translate(translation);
    transformation_matrix.rotate(rotation);
    cv::Mat cv_transformation_matrix = cv::Mat::eye(4, 4, CV_64F);
    cv::eigen2cv(transformation_matrix.matrix(), cv_transformation_matrix);

    return cv_transformation_matrix;
  }

  cv::Point3d eigenVector3dToCvPoint3d(const Eigen::Vector3d& vec) { return cv::Point3d(vec.x(), vec.y(), vec.z()); }

}  // namespace

namespace vslam_backend_plugins {
  Indirect::~Indirect() {
    exit_thread_ = true;

    if (local_ba_thread_.joinable()) {
      local_ba_condition_.notify_one();
      local_ba_thread_.join();
    }
  }

  void Indirect::initialize(const cv::Mat& K, const FrameQueue::SharedPtr frame_queue) {
    K_ = K;
    frame_queue_ = frame_queue;
    local_ba_thread_ = std::thread(&Indirect::local_ba_loop, this);
  }

  void Indirect::add_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    assert(keyframe->is_keyframe());

    keyframes_[keyframe->id()] = keyframe;

    current_keyframe_ = keyframe;

    {
      std::lock_guard<std::mutex> lck(local_ba_mutex_);
      run_local_ba_ = true;
      local_ba_condition_.notify_one();
    }
  }

  void Indirect::remove_keyframe(vslam_datastructure::Frame::SharedPtr keyframe) {
    if (keyframes_.find(keyframe->id()) != keyframes_.end()) {
      keyframes_.erase(keyframe->id());
    }
  }

  vslam_datastructure::Frame::SharedPtr Indirect::get_current_keyframe() { return current_keyframe_; }

  void Indirect::local_ba_loop() {
    while (!exit_thread_) {
      {
        std::unique_lock<std::mutex> lck(local_ba_mutex_);
        local_ba_condition_.wait(lck, [this] { return exit_thread_ || run_local_ba_; });
      }

      if (exit_thread_) {
        break;
      }

      auto [core_kfs, core_mps] = get_core_keyframes_mappoints();
      if (core_kfs.size() < num_core_kfs_) {
        continue;
      }

      // run_local_ba(core_kfs, core_mps);

      run_local_ba_ = false;
    }
    std::cout << "terminated local BA" << std::endl;
  }

  std::pair<Indirect::CoreKfsSet, Indirect::CoreMpsSet> Indirect::get_core_keyframes_mappoints() {
    assert(num_core_kfs_ > 0);

    CoreKfsSet core_keyframes;
    CoreMpsSet core_mappoints;
    auto kfs_it = keyframes_.rbegin();
    // for (size_t i = 0; i < num_core_kfs_; i++) {
    while (kfs_it != keyframes_.rend() && core_keyframes.size() < num_core_kfs_) {
      auto kf = kfs_it->second;

      if (kf.get() == nullptr) {
        continue;
      }

      core_keyframes.insert(kf.get());

      // TODO: make it thread-safe
      for (auto pt : kf->get_points()) {
        if (pt->mappoint.get() && !pt->mappoint->is_outlier() && pt->mappoint->get_projections().size() > 1) {
          core_mappoints.insert(pt->mappoint.get());
        }
      }

      kfs_it++;
    }

    return {core_keyframes, core_mappoints};
  }

  void Indirect::run_local_ba(CoreKfsSet& core_keyframes, CoreMpsSet& core_mappoints) {
    // Setup g2o optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver
        = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver
        = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver)));
    optimizer.setAlgorithm(solver);
    // std::string solver_name;
    // // #ifdef G2O_HAVE_CHOLMOD
    // //     solver_name = "lm_fix6_3_cholmod";
    // // #else
    // solver_name = "lm_fix6_3";
    // // #endif;
    // g2o::OptimizationAlgorithmProperty solver_property;
    // optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(solver_name, solver_property));

    constexpr float huber_kernel_delta{2.45};

    // Create vertices and edges
    std::map<g2o::VertexPointXYZ*, vslam_datastructure::MapPoint*> core_mp_vertices;
    std::map<g2o::VertexSE3Expmap*, vslam_datastructure::Frame*> core_kf_vertices;
    std::list<g2o::EdgeSE3ProjectXYZ*> all_edges;
    std::map<vslam_datastructure::Frame*, g2o::VertexSE3Expmap*> existing_kf_vertices;
    unsigned long int vertex_id{0};
    for (auto mp : core_mappoints) {
      if (mp == nullptr || mp->is_outlier() || mp->get_projections().empty()) {
        continue;
      }

      g2o::VertexPointXYZ* mp_vertex = new g2o::VertexPointXYZ();
      core_mp_vertices[mp_vertex] = mp;
      mp_vertex->setEstimate(cvPoint3dToEigenVector3d(mp->get_mappoint()));
      mp_vertex->setId(vertex_id++);
      mp_vertex->setMarginalized(true);
      optimizer.addVertex(mp_vertex);

      // Projections
      for (auto pt : mp->get_projections()) {
        if (pt == nullptr || pt->frame == nullptr) {
          continue;
        }

        // Keyframe vertex
        g2o::VertexSE3Expmap* kf_vertex;
        if (existing_kf_vertices.find(pt->frame) != existing_kf_vertices.end()) {
          kf_vertex = existing_kf_vertices[pt->frame];
        } else {
          kf_vertex = new g2o::VertexSE3Expmap();
          existing_kf_vertices[pt->frame] = kf_vertex;
          kf_vertex->setEstimate(cvMatToSE3Quat(pt->frame->get_pose()));
          kf_vertex->setId(vertex_id++);
          if (core_keyframes.find(pt->frame) != core_keyframes.end()) {
            // Core keyframes
            core_kf_vertices[kf_vertex] = pt->frame;
            kf_vertex->setFixed(false);
          } else {
            // Non-core keyframes
            kf_vertex->setFixed(true);
          }
          optimizer.addVertex(kf_vertex);
        }

        // Projection
        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
        e->setVertex(0, mp_vertex);
        e->setVertex(1, kf_vertex);
        e->setMeasurement(Eigen::Vector2d(pt->keypoint.pt.x, pt->keypoint.pt.y));
        e->setInformation(Eigen::Matrix2d::Identity());

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(huber_kernel_delta);

        e->fx = K_.at<double>(0, 0);
        e->fy = K_.at<double>(1, 1);
        e->cx = K_.at<double>(0, 2);
        e->cy = K_.at<double>(1, 2);

        optimizer.addEdge(e);
        all_edges.push_back(e);
      }
    }

    std::cout << "Num kf vertices: " << core_kf_vertices.size() << std::endl;
    std::cout << "Num mp vertices: " << core_mp_vertices.size() << std::endl;

    // optimize graph
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // // TODO: only update the core keyframes and map points
    // // Update keyframes
    // for (auto [kf_vertex, kf_p] : core_kf_vertices) {
    //   auto pose = kf_vertex->estimate();
    //   cv::Mat cv_pose = eigenRotationTranslationToCvMat(pose.rotation().toRotationMatrix(), pose.translation());
    //   if (kf_p) {
    //     kf_p->set_pose(cv_pose);
    //   }
    // }

    // for (auto e : all_edges) {
    //   if (!e->isDepthPositive() || e->chi2() > 5.991) {
    //     auto mp_vertex = static_cast<g2o::VertexPointXYZ*>(e->vertex(0));
    //     auto mp_p = core_mp_vertices[mp_vertex];

    //     if (mp_p) {
    //       mp_p->is_outlier = true;
    //     }
    //   }
    // }

    // // Update map points
    // for (auto [mp_vertex, mp_p] : core_mp_vertices) {
    //   auto mp = mp_vertex->estimate();
    //   if (mp.hasNaN()) {
    //     continue;
    //   }

    //   if (mp_p && !mp_p->is_outlier) {
    //     auto pt_3d = eigenVector3dToCvPoint3d(mp);

    //     // The difference before and after optimization cannot be too large
    //     if (cv::norm(pt_3d - mp_p->pt_3d) < 1.5) {
    //       mp_p->pt_3d = pt_3d;
    //     }

    //     // std::cout << "before: " << mp_p->pt_3d.x << " " << mp_p->pt_3d.y << " " << mp_p->pt_3d.z << std::endl;
    //     // std::cout << "after: " << mp.transpose() << std::endl;
    //     // mp_p->pt_3d.x = mp.x();
    //     // mp_p->pt_3d.y = mp.y();
    //     // mp_p->pt_3d.z = mp.z();
    //   }
    // }
  }

}  // namespace vslam_backend_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(vslam_backend_plugins::Indirect, vslam_backend_base::Backend)