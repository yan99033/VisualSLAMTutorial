// TODO: Licence

#ifndef VSLAM_NODES__PLUGIN_FACTORY_HPP_
#define VSLAM_NODES__PLUGIN_FACTORY_HPP_

#include <pluginlib/class_loader.hpp>

#include "vslam_plugins_base/backend.hpp"
#include "vslam_plugins_base/camera_tracker.hpp"
#include "vslam_plugins_base/feature_extractor.hpp"
#include "vslam_plugins_base/feature_matcher.hpp"
#include "vslam_plugins_base/mapper.hpp"
#include "vslam_plugins_base/place_recognition.hpp"
#include "vslam_plugins_base/visualizer.hpp"

namespace vslam_components {
  namespace vslam_nodes {
    namespace vslam_plugins {

      class Loader {
      public:
        std::shared_ptr<vslam_feature_extractor_base::FeatureExtractor> feature_extractor(
            const std::string& class_name) {
          return feature_extractor_loader_.createSharedInstance(class_name);
        }

        std::shared_ptr<vslam_feature_matcher_base::FeatureMatcher> feature_matcher(const std::string& class_name) {
          return feature_matcher_loader_.createSharedInstance(class_name);
        }

        std::shared_ptr<vslam_camera_tracker_base::CameraTracker> camera_tracker(const std::string& class_name) {
          return camera_tracker_loader_.createSharedInstance(class_name);
        }

        std::shared_ptr<vslam_mapper_base::Mapper> mapper(const std::string& class_name) {
          return mapper_loader_.createSharedInstance(class_name);
        }

        std::shared_ptr<vslam_backend_base::Backend> backend(const std::string& class_name) {
          return backend_loader_.createSharedInstance(class_name);
        }

        std::shared_ptr<vslam_place_recognition_base::PlaceRecognition> place_recognition(
            const std::string& class_name) {
          return place_recognition_loader_.createSharedInstance(class_name);
        }

        std::shared_ptr<vslam_visualizer_base::Visualizer> visualizer(const std::string& class_name) {
          return visualizer_loader_.createSharedInstance(class_name);
        }

      private:
        pluginlib::ClassLoader<vslam_feature_extractor_base::FeatureExtractor> feature_extractor_loader_{
            "vslam_plugins_base", "vslam_feature_extractor_base::FeatureExtractor"};

        pluginlib::ClassLoader<vslam_feature_matcher_base::FeatureMatcher> feature_matcher_loader_{
            "vslam_plugins_base", "vslam_feature_matcher_base::FeatureMatcher"};

        pluginlib::ClassLoader<vslam_camera_tracker_base::CameraTracker> camera_tracker_loader_{
            "vslam_plugins_base", "vslam_camera_tracker_base::CameraTracker"};

        pluginlib::ClassLoader<vslam_mapper_base::Mapper> mapper_loader_{"vslam_plugins_base",
                                                                         "vslam_mapper_base::Mapper"};

        pluginlib::ClassLoader<vslam_backend_base::Backend> backend_loader_{"vslam_plugins_base",
                                                                            "vslam_backend_base::Backend"};

        pluginlib::ClassLoader<vslam_place_recognition_base::PlaceRecognition> place_recognition_loader_{
            "vslam_plugins_base", "vslam_place_recognition_base::PlaceRecognition"};

        pluginlib::ClassLoader<vslam_visualizer_base::Visualizer> visualizer_loader_{
            "vslam_plugins_base", "vslam_visualizer_base::Visualizer"};
      };

    }  // namespace vslam_plugins
  }    // namespace vslam_nodes
}  // namespace vslam_components

#endif  // VSLAM_NODES__PLUGIN_FACTORY_HPP_