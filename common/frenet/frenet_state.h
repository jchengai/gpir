/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>

namespace common {

struct FrenetPoint {
  double s = 0.0;
  double d = 0.0;

  FrenetPoint() = default;
  FrenetPoint(const double s, const double d) : s(s), d(d) {}

  std::string DebugString() const {
    return "[FrenetPoint] s: " + std::to_string(s) +
           ", d: " + std::to_string(d);
  }
};

struct FrenetReferencePoint {
  Eigen::Vector2d point;
  double s = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;

  std::string DebugString() const {
    std::ostringstream os;
    os << "[FrenetReferencePoint]:\n  "
       << "point: (" << point.x() << ", " << point.y() << ")\n  "
       << "s: " << s << "\n  "
       << "theta: " << theta << "\n  "
       << "kappa: " << kappa << "\n  "
       << "dkappa: " << dkappa << "\n";
    return os.str();
  }
};

struct FrenetState {
  FrenetState() = default;
  FrenetState(const Eigen::Vector3d& s, const Eigen::Vector3d& d)
      : s(s), d(d) {}

  enum class DerivativeType {
    kDs = 0,
    kDt = 1,
  };

  Eigen::Vector3d s;
  Eigen::Vector3d d;
  DerivativeType type = DerivativeType::kDs;

  FrenetPoint frenet_point() { return FrenetPoint(s[0], d[0]); }

  std::string DebugString() const {
    std::ostringstream os;
    os << "[FrenetState]:\n  "
       << "s: (" << s[0] << ", " << s[1] << ", " << s[2] << ")\n  "
       << "d: (" << d[0] << ", " << d[1] << ", " << d[2] << ")\n";
    return os.str();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common