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

    /// Send a message
    /**
     * \param[in] msg message
     */
    void send(T&& msg) {
      std::lock_guard<std::mutex> lck(mutex_);
      queue_.push_back(std::move(msg));

      condition_.notify_one();
    }

    /// Stop and exit
    void stop() {
      // Unblock the condition variable
      exit_ = true;
      condition_.notify_one();
    }

    /// Receive a message
    /**
     * \return a messsage in the queue
     */
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
    /// A queue used to store the messages
    std::deque<T> queue_;

    /// Condition variable to notify if a message is received
    std::condition_variable condition_;

    /// Mutex for reading and writing the queue
    std::mutex mutex_;

    /// A flag to clear the queue and exit
    std::atomic_bool exit_{false};
  };
  using FrameIdQueue = SignalQueue<long unsigned int>;

}  // namespace vslam_datastructure
#endif  // VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_