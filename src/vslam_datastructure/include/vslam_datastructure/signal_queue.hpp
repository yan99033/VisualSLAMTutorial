#ifndef VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_
#define VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>

#include "vslam_msgs/msg/frame.hpp"

namespace vslam_datastructure {
  template <typename T> class SignalQueue {
  public:
    using SharedPtr = std::shared_ptr<SignalQueue>;

    void send(T&& msg) {
      std::lock_guard<std::mutex> lck(mutex_);
      queue_.push_back(std::move(msg));

      condition_.notify_one();
    }

    void send_front(T&& msg) {
      std::lock_guard<std::mutex> lck(mutex_);
      queue_.push_front(std::move(msg));

      condition_.notify_one();
    }

    void stop() {
      // Unblock the condition variable
      exit_ = true;
      condition_.notify_one();
    }

    T receive() {
      std::unique_lock<std::mutex> lck(mutex_);
      condition_.wait(lck, [this] { return !queue_.empty() || exit_; });

      if (exit_) {
        queue_.clear();
        return T();
      }

      if (queue_.empty()) {
        return T();
      }

      // Remove the first element in the queue (FIFO)
      T msg = std::move(queue_.front());
      queue_.pop_front();

      return msg;
    }

  private:
    std::deque<T> queue_;
    std::condition_variable condition_;
    std::mutex mutex_;

    std::atomic_bool exit_{false};
  };
  using FrameIdQueue = SignalQueue<long unsigned int>;

  class FrameMsgQueue {
  public:
    using SharedPtr = std::shared_ptr<FrameMsgQueue>;

    void send(vslam_msgs::msg::Frame&& msg) {
      std::lock_guard<std::mutex> lck(mutex_);
      map_[msg.id] = std::move(msg);
      condition_.notify_one();
    }

    void stop() {
      // Unblock the condition variable
      exit_ = true;
      condition_.notify_one();
    }

    vslam_msgs::msg::Frame receive() {
      std::unique_lock<std::mutex> lck(mutex_);
      condition_.wait(lck, [this] { return !map_.empty() || exit_; });

      if (exit_) {
        map_.clear();
        return vslam_msgs::msg::Frame();
      }

      if (map_.empty()) {
        return vslam_msgs::msg::Frame();
      }

      // Remove the last element in the map
      vslam_msgs::msg::Frame msg = std::move(std::prev(map_.end())->second);
      map_.erase(std::prev(map_.end()));

      return msg;
    }

  private:
    std::map<long unsigned int, vslam_msgs::msg::Frame, std::less<long unsigned int>> map_;
    std::condition_variable condition_;
    std::mutex mutex_;

    std::atomic_bool exit_{false};
  };

}  // namespace vslam_datastructure
#endif  // VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_