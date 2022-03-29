/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <chrono>
#include <iostream>
#include <string>

#include "common/utils/stdout_color.h"

namespace common {

class Timer {
 public:
  Timer() { Start(); };
  Timer(const double timeout) : timeout_(timeout) {}

  inline void Start();
  inline void Reset();
  inline double End();           // in millseconds
  inline double EndThenReset();  // in millseconds
  inline void End(const std::string& description);

  inline void set_timeout(const double timeout_ms) { timeout_ = timeout_ms; }
  inline bool timeout() { return End() > timeout_; }

 private:
  double timeout_ = 0.0;  // ms
  std::chrono::time_point<std::chrono::system_clock> start_time_;
};

void Timer::Start() { start_time_ = std::chrono::high_resolution_clock::now(); }

void Timer::Reset() { Start(); }

inline double Timer::End() {
  using namespace std::chrono;
  auto end_time = high_resolution_clock::now();
  return duration_cast<nanoseconds>(end_time - start_time_).count() / 1e6;
}

inline double Timer::EndThenReset() {
  using namespace std::chrono;
  double duration =
      duration_cast<nanoseconds>(high_resolution_clock::now() - start_time_)
          .count() /
      1e6;
  Start();
  return duration;
}

inline void Timer::End(const std::string& description) {
  double t = End();
  // fprintf(stdout, "%s[%s]%s takes %f ms\n", ANSI_BLUE, description.c_str(),
  //         ANSI_RESET, t);
}

#define TIC ::common::Timer timer
#define TOC(description) timer.End(description);
}  // namespace common
