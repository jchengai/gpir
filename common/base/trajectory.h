/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <vector>
#include <iostream>

#include "common/base/state.h"
#include "common/utils/math.h"

namespace common {

class Trajectory : public std::vector<common::State> {
 public:
  Trajectory() = default;
  virtual ~Trajectory() = default;

  int GetNearsetIndex(const Eigen::Vector2d& pos) const {
    int min_index = 0;
    double dis = 0;
    double min_dis = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < size(); ++i) {
      dis = (data()[i].position - pos).squaredNorm();
      if (dis < min_dis) {
        min_index = i;
        min_dis = dis;
      }
    }
    return min_index;
  }

  const common::State& GetNearestState(const Eigen::Vector2d& pos) const {
    return data()[GetNearsetIndex(pos)];
  }

  const common::State GetInterpolatedNearestState(
      const Eigen::Vector2d& pos) const {
    auto index = GetNearsetIndex(pos);
    const auto& near_state = data()[index];
    auto tanget = Eigen::Vector2d(std::cos(near_state.heading),
                                  std::sin(near_state.heading));
    Eigen::Vector2d vec = pos - near_state.position;
    auto next_index = index;
    if (vec.dot(tanget) >= 0) {
      next_index += 1;
    } else {
      next_index = index;
      index = std::max(0, index - 1);
    }
    // std::cout << "wtf??" << std::endl;

    const auto& s0 = data()[index];
    const auto& s1 = data()[next_index];
    double w =
        (pos.x() - s0.position.x()) / (s1.position.x() - s0.position.x());
    common::State inter_state;
    inter_state.position = (1 - w) * s0.position + w * s1.position;
    inter_state.heading = common::InterpolateAngle(
        s0.heading, s0.position.x(), s1.heading, s1.position.x(), pos.x());
    inter_state.kappa = (1 - w) * s0.kappa + w * s1.kappa;
    inter_state.velocity = (1 - w) * s0.velocity + w * s1.velocity;
    // std::cout << "wtf???" << std::endl;
    return inter_state;
  }
};

inline std::ostream& operator<<(std::ostream& os,
                                const Trajectory& trajectory) {
  os << "Trajectory: \n";
  for (const auto& point : trajectory) {
    os << std::fixed << std::setprecision(4) << "[" << std::left
       << "x: " << std::setw(6) << point.position.x() << ", "
       << "y: " << std::setw(6) << point.position.y() << ", "
       << "s: " << std::setw(6) << point.s << ", "
       << "v: " << std::setw(6) << point.velocity << ", "
       << "a: " << std::setw(6) << point.acceleration << "]\n";
  }
  return os;
}

}  // namespace common