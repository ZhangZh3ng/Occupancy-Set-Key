/*
  Author: Zheng Zhang
  Created: 2023-4-12
  Description: A simple timer
*/

#pragma once

#include <chrono>

// Timekeeper is used for measuring elapsed time with high precision
// using the C++ standard library's chrono library. It can be paused and
// resumed, and any time during the pause is excluded from the total elapsed
// time.
class Timekeeper {
 public:
  using TimePoint = decltype(std::chrono::high_resolution_clock::now());

  // Constructor that sets the start time, elapsed time, and running status
  Timekeeper()
      : start_time_(std::chrono::high_resolution_clock::now()),
        elapsed_time_(0), last_elapsed_time_(0),
        is_running_(false) {}

  // Resets the start time and elapsed time, and sets running status to false
  void Reset() {
    start_time_ = std::chrono::high_resolution_clock::now();
    elapsed_time_ = 0;
    last_elapsed_time_ = 0;
    is_running_ = false;
  }

  // Stops the timer and records the last elapsed time
  void Pause() {
    if (is_running_) {
      last_elapsed_time_ =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::high_resolution_clock::now() - start_time_)
              .count();
      elapsed_time_ += last_elapsed_time_;
      is_running_ = false;
    }
  }

  // Starts the timer if it's stopped and updates the start time
  void Resume() {
    if (!is_running_) {
      start_time_ = std::chrono::high_resolution_clock::now();
      is_running_ = true;
    }
  }

  // Returns whether the timer is currently running
  bool is_running() const { return is_running_; }

  // Returns the elapsed time in microseconds (1e-6 seconds) since the last
  // pause to resume. If the timer is currently running, the end time point is
  // the current time.
  double GetLastElapsedTime() const {
    if (is_running_) {
      return std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::high_resolution_clock::now() - start_time_)
                 .count();
    } else {
      return last_elapsed_time_;
    }
  }

  // Returns the overall elapsed time in microseconds (1e-6 seconds)
  double GetOverallTime() const {
    if (is_running_) {
      auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now() - start_time_)
                    .count();
      return elapsed_time_ + dt;
    } else {
      return elapsed_time_;
    }
  }

 private:
  // The time point when the timer was started or last resumed
  TimePoint start_time_;
  // The total elapsed time (excluding any time when the timer was paused)
  double elapsed_time_;
  // The time elapsed during the last resume to pause
  double last_elapsed_time_;
  // Whether the timer is currently running
  bool is_running_;
};
