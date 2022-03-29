/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <cmath>

namespace planning {

class PenaltyFunction {
 public:
  PenaltyFunction() = default;
  PenaltyFunction(const double limit, double alpha = 1.0)
      : limit_(limit), alpha_(alpha) {}

  double EvaluateHinge(const double val, double* grad) const {
    double cost = 0.0;
    if (val > limit_) {
      *grad = 1;
      cost = (val - limit_);
    } else if (val < -limit_) {
      *grad = -1;
      cost = (-limit_ - val);
    } else {
      *grad = 0.0;
    }
    return cost;
  }

  double EvaluatePoly(const double val, double* grad) const {
    double cost = 0.0;
    if (val > limit_) {
      double diff = val - limit_;
      *grad = 2 * diff;
      cost = diff * diff;
    } else if (val < -limit_) {
      double diff = -limit_ - val;
      *grad = -2 * diff;
      cost = diff * diff;
    } else {
      *grad = 0.0;
    }
    return cost;
  }

  double EvaluateCubic(const double val, double* grad, double eps) const {
    double limit1_ = limit_;
    double limit2_ = limit_ + eps;
    double a2_ = 3 * eps;
    double b2_ = 3 * eps * eps - 2 * a2_ * limit2_;
    double c2_ = eps * eps * eps - a2_ * limit2_ * limit2_ - b2_ * limit2_;
    double a1_ = 3 * eps;
    double b1_ = -3 * eps * eps + 2 * a1_ * limit2_;
    double c1_ = eps * eps * eps - a1_ * limit2_ * limit2_ + b1_ * limit2_;
    double cost = 0.0;

    if (std::fabs(val) < limit1_) {
      *grad = 0.0;
      cost = 0.0;
    } else if (limit1_ <= val && val < limit2_) {
      double error = val - limit1_;
      *grad = 3 * error * error;
      cost = error * error * error;
    } else if (limit2_ <= val) {
      *grad = 2 * a2_ * val + b2_;
      cost = a2_ * val * val + b2_ * val + c2_;
    } else if (-limit2_ < val && val <= -limit1_) {
      double error = -limit1_ - val;
      *grad = -3 * error * error;
      cost = error * error * error;
    } else {
      *grad = 2 * a1_ * val + b1_;
      cost = a1_ * val * val + b1_ * val + c1_;
    }
    return cost;
  }

 private:
  double alpha_ = 1.0;
  double limit_ = 0.0;
};

}  // namespace planning
