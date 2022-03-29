/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <deque>

namespace common {

class BoxCarFilter {
 public:
  BoxCarFilter() = default;
  BoxCarFilter(const int window_size) : window_size_(window_size) {}

  double Update(const double input) {
    int size = inputs_.size();
    if (size < window_size_) {
      previous_val_ = (previous_val_ * size + input) / (size + 1);
    } else {
      double oldest_val = inputs_.front();
      previous_val_ = previous_val_ + (input - oldest_val) / window_size_;
      inputs_.pop_front();
    }
    inputs_.push_back(input);
    return previous_val_;
  }

  void Rest() {
    previous_val_ = 0.0;
    inputs_.clear();
  }

 private:
  int window_size_ = 5;
  double previous_val_ = 0.0;
  std::deque<double> inputs_;
};

}  // namespace common
