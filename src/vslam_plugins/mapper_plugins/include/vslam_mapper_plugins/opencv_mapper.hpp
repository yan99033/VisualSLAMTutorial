#ifndef VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
#define VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_

#include <opencv2/calib3d.hpp>

#include "vslam_datastructure/point.hpp"
#include "vslam_plugins_base/mapper.hpp"

namespace vslam_mapper_plugins {

  class OpenCVMapper : public virtual vslam_mapper::base::Mapper {
  public:
    /// Destructor
    ~OpenCVMapper() { std::cerr << "Terminated OpenCVMapper" << std::endl; }

    /// Mapper initializer
    void initialize() override {}

    /// Create new map points based on the point correspondences and the pose constraints
    /**
     * \param[in] matched_points Point correspndences
     * \param[in] T_1_w camera pose of the first frane
     * \param[in] T_2_1 camera pose from the first to second frame
     * \param[in] K camera matrix
     */
    virtual vslam_datastructure::MapPoints map(vslam_datastructure::MatchedPoints& matched_points, const cv::Mat& T_1_w,
                                               const cv::Mat& T_2_1, const cv::Mat& K) override;

    /// Get the plugin name
    inline std::string getPluginName() override { return "vslam_mapper_plugins::OpenCVMapper"; }

  private:
    /// Threshold to remove map points that have a large reprojection error
    static constexpr const double proj_err_thresh_{4.0};
  };
}  // namespace vslam_mapper_plugins

#endif  // VSLAM_MAPPER_PLUGINS__OPENCV_TRIANGULATION_HPP_
