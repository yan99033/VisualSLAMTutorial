# Visual SLAM tutorial (beta)

A visual SLAM (simultaneous localization and mapping) framework provides the foundation for integrating different components within it. 
For example, A visual SLAM system comprises camera tracking, mapping, loop closing via place recognition, and visualization components. 
The framework connects the components such that we get the camera motion and the structure of the environment from a stream of images in real-time.

## Objective

The main goal of this project is to create an ease-of-use framework to learn visual SLAM while allowing for customizability for better performance.
To this end, we define the components in the framework as plugins, which can be modified so long as the interfaces (i.e., the inputs and outputs) are unchanged.

## Framework

We organize the visual SLAM components as individual plugins/libraries to test them in isolation before integrating them into the framework. 
The plugins are loaded into ROS2 [composable nodes](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) to run them in a single process. The nodes and plugins are described as follows:
- **data loader node** takes a list of images from a folder or an image stream from a camera and converts them into the Frame message, which contains the image and the camera information.

- **vslam node** processes the incoming Frame messages to track the camera motion and structure of the scene. The system begins in the initialization state, where we set the current frame as the tentative keyframe and attempt initialization in the next frame. Any failure in tracking and mapping resets the state back to initialization. Once the system is initialized, we track the camera pose of the images against the keyframe and create new keyframe and map points (structure of the scene) as needed. In a parallel thread, we check if the camera revisits a previously mapped area and optimize the camera motion and structure globally. In the event of failure to find correspondences between two subsequent frames, the system enters the relocalization state, where we attempt to regain camera tracking. This node consists of seven plugins:
  - **feature extraction plugin** calculates the keypoints (e.g., corners or high-gradient image regions) and, optionally, their descriptors in the image.
  - **feature matcher plugin** finds feature correspondences between the current frame and the nearest keyframe in the back-end during the tracking state. 
  - **camera tracker plugin** calculates the relative pose between the current frame and the current keyframe. 
  - **mapper plugin** triangulates a set of new map points based on the relative pose and the feature correspondences.
  - **place recognition plugin** finds the keyframe image similar to the current frame image.
  - **back-end plugin** runs local bundle adjustment and pose-graph optimizations.
  - **visualizer plugin** gets the Frame messages and update the visualizer accordingly.

## Setup

- The code has been tested on Ubuntu and macOS running ROS 2 Humble
  - [Installation instructions for Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  - [Installation instructions for macOS](https://github.com/RoboStack/ros-humble)

- Download [KITTI odometry dataset (color, 65 GB)](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)

## Build

Run `colcon build` to build the packages.

### Run 
- Move the `orbvoc.dbow3` vocabulary file from [build/thirdparty/_deps/dbow3-src](build/thirdparty/_deps/dbow3-src) to the root directory
- Create a copy of the [KITTI camera parameters](src/camera_plugins//monocular_camera_plugins/params/kitti_camera.yaml.example) (and remove the .example extension) and modify the parameters accordingly.
- Create a copy of the [vslam_demo parameters](src/vslam_demos/params/test_kitti.yaml.example) (and remove the .example extension) and modify the parameters accordingly.
- 
- Launch the demo
  ```bash
  source install/setup.bash
  ros2 launch vslam_demos vslam_from_folder.launch.py params:=<path_to_test_kitti_yaml_file>
  ```
  > Replace the first command with `source install/setup.zsh` if you are using macOS. 