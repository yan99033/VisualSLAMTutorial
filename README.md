# Visual SLAM tutorial

A visual SLAM (simultaneous localization and mapping) framework provides the foundation for integrating different components within it. 
For example, A visual SLAM system comprises camera tracking, mapping, loop closing via place recognition, and visualization components. 
The framework connects the components such that we get the camera motion and the structure of the environment from a stream of images in real-time.

## Objective

The main goal of this project is to create an ease-of-use framework to learn visual SLAM while allowing for customizability for better performance.
To this end, we define the components in the framework as plugins, which can be modified so long as the interfaces (i.e., the inputs and outputs) are unchanged.
I will provide a one-year commitment (TODO: insert date here) to support the community, hoping the framework will incorporate more features and improve through optimization and fixing bugs.

## Framework


If the system is in the initialization state, we set the current frame as the tentative keyframe and attempt initialization in the next frame. Any failure in tracking and mapping will reset the state back to initialization. Once the system is initialized, we track the camera pose of the images against the keyframe and create new keyframe and map points (structure of the scene) as needed. If ca
  If the system is in the relocalization mode, the current keyframe image is published to the place recognition node to find a candidate keyframe index, which is used to request the keyframe in the back-end to perform feature matching. In the event of failure to find correspondences, the node will set the state to relocalization.


To promote flexibility, we can organize the visual SLAM components as individual plugins/libraries to test them in isolation before integrating them into the framework. 
We load the plugins into ROS2 [composable nodes](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) to run them in a single process. The nodes and plugins are described as follows:
- **data loader node** takes a list of images from a folder or an image stream from a camera and converts them into the Frame message, which contains the image and the camera information.

- **vslam node** processes the incoming Frame messages to track the camera motion and structure of the scene. The system begins in the initialization state, where we set the current frame as the tentative keyframe and attempt initialization in the next frame. Any failure in tracking and mapping resets the state back to initialization. Once the system is initialized, we track the camera pose of the images against the keyframe and create new keyframe and map points (structure of the scene) as needed. In a parallel thread, we check if the camera revisits a previously mapped area and optimize the camera motion and structure globally. In the event of failure to find correspondences between two subsequent frames, the system enters the relocalization state, where we attempt to regain camera tracking. This node consists of six plugins:
  - **feature extraction plugin** calculates the keypoints (e.g., corners or high-gradient image regions) and, optionally, their descriptors in the image.
  - **feature matching plugin** finds feature correspondences between the current frame and the nearest keyframe in the back-end during the tracking state. 



## Usage

### Build the libraries incrementally

### Build and run the standalone target

Use the following command to build and run the executable target.

```bash
cmake -S standalone -B build/standalone
cmake --build build/standalone
./build/standalone/Greeter --help
```

## Acknowledgement

This repository is created using [ModernCppStarter](https://github.com/TheLartians/ModernCppStarter): A template for modern C++ project
