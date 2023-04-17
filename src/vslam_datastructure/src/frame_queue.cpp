#include "vslam_datastructure/frame_queue.hpp"

namespace vslam_datastructure {
  void FrameQueue::send(vslam_msgs::msg::Frame &&msg) {
    std::lock_guard<std::mutex> lck(mutex_);
    queue_.push_back(std::move(msg));

    condition_.notify_one();
  }

  void FrameQueue::stop() {
    // Unblock the condition variable
    exit_ = true;
    condition_.notify_one();
  }

  vslam_msgs::msg::Frame FrameQueue::receive() {
    std::unique_lock<std::mutex> lck(mutex_);
    condition_.wait(lck, [this] { return !queue_.empty() || !exit_; });

    if (exit_) {
      queue_.clear();
      return vslam_msgs::msg::Frame();
    }

    if (queue_.empty()) {
      return vslam_msgs::msg::Frame();
    }

    // Remove the first element in the queue (FIFO)
    vslam_msgs::msg::Frame msg = std::move(queue_.front());
    queue_.pop_front();

    return msg;
  }
}  // namespace vslam_datastructure