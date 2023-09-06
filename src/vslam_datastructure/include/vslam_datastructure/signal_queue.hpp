/**
 * This file is part of VisualSLAMTutorial
 *
 * Copyright (C) 2023  Shing-Yan Loo <yan99033 at gmail dot com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_
#define VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>

#include "vslam_datastructure/frame.hpp"
#include "vslam_msgs/msg/frame.hpp"

namespace vslam_datastructure {
  template <typename T> class SignalQueue {
  public:
    using SharedPtr = std::shared_ptr<SignalQueue>;

    ~SignalQueue() {
      queue_.clear();
      std::cerr << "Terminated SignalQueue" << std::endl;
    }

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
}  // namespace vslam_datastructure
#endif  // VSLAM_DATASTRUCTURE__SIGNAL_QUEUE_HPP_