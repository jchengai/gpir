/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <cmath>
#include <iostream>

namespace planning {

class BoundedPenaltyFunction {
 public:
  BoundedPenaltyFunction() = default;
  BoundedPenaltyFunction(const double limit, const double eps) {
    limit1_ = limit;
    limit2_ = limit + eps;
    a2_ = 3 * eps;
    b2_ = 3 * eps * eps - 2 * a2_ * limit2_;
    c2_ = eps * eps * eps - a2_ * limit2_ * limit2_ - b2_ * limit2_;

    a1_ = 3 * eps;
    b1_ = -3 * eps * eps + 2 * a1_ * limit2_;
    c1_ = eps * eps * eps - a1_ * limit2_ * limit2_ + b1_ * limit2_;
  }

  double GetPenaltyAndGradient(const double val, double* grad) const {
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
    (*grad) *=1000;
    cost *= 1000;
    return cost;
  }

  //  private:
  double limit1_ = 0.0;
  double limit2_ = 0.0;
  double eps_ = 0.0;

  double a1_ = 0.0;
  double b1_ = 0.0;
  double c1_ = 0.0;
  double a2_ = 0.0;
  double b2_ = 0.0;
  double c2_ = 0.0;
};

}  // namespace planning
