#ifndef VSLAM_DATASTRUCTURE__FRAME_QUEUE_HPP_
#define VSLAM_DATASTRUCTURE__FRAME_QUEUE_HPP_

#include <deque>

#include "vslam_msgs/msg/frame.hpp"

class FrameQueue {
public:
  void send(vslam_msgs::msg::Frame &&msg);
  void stop();
  vslam_msgs::msg::Frame receive();

private:
  std::deque<vslam_msgs::msg::Frame> queue_;
  std::condition_variable condition_;
  std::mutex mutex_;

  std::atomic_bool exit_{false};
};

#endif