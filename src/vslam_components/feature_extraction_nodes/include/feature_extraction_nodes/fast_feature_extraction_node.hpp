// TODO: Licence

#ifndef FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_
#define FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_components {
  namespace feature_extraction_nodes {
    class FastFeatureExtractionNode : public rclcpp::Node {
    public:
      explicit FastFeatureExtractionNode(const rclcpp::NodeOptions &options);

    private:
      void frame_callback(const vslam_msgs::msg::Frame::SharedPtr frame_msg) const;
    
      rclcpp::Subscription<vslam_msgs::msg::Frame>::SharedPtr frame_sub_;
      rclcpp::Publisher<vslam_msgs::msg::Frame>::SharedPtr frame_pub_;
    };
  }
}

#endif // FEATURE_EXTRACTION_NODES__FAST_EXTRACTOR_HPP_