/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/initializer/gp_initializer.h"

#include "common/smoothing/osqp_spline1d_solver.h"

namespace planning {

void GPInitializer::SetBoundary(
    std::vector<std::vector<std::pair<double, double>>> boundary) {
  // tmp test
  boundary_.clear();
  for (const auto b : boundary) {
    auto a = b.front();
    a.first += 2.5;
    a.second -= 2.5;
    // std::cout << a.first << ", " << a.second << std::endl;
    boundary_.emplace_back(a);
  }
}

bool GPInitializer::GenerateInitialPath(
    const Eigen::Vector3d& x0, const Eigen::Vector3d& xn,
    const std::vector<double> s_refs,
    const std::vector<double>& obstacle_location_hint,
    const std::vector<double>& lb, std::vector<double>& ub,
    vector_Eigen3d* result) {
  double start = s_refs.front();
  double length = s_refs.back() - start;
  std::vector<double> knots{start, start + length / 4.0, start + length / 2.0,
                            start + length * 3.0 / 4.0, start + length};

  common::OsqpSpline1dSolver spline1d_solver(knots, 5);

  std::vector<double> ref(s_refs.size(), 0);

  auto mutable_kernel = spline1d_solver.mutable_kernel();
  auto mutable_constraint = spline1d_solver.mutable_constraint();
  mutable_kernel->AddReferenceLineKernelMatrix(s_refs, ref, 1);
  mutable_kernel->AddSecondOrderDerivativeMatrix(200);
  mutable_kernel->AddThirdOrderDerivativeMatrix(1000);
  mutable_constraint->AddPointConstraint(start, x0(0));
  mutable_constraint->AddPointDerivativeConstraint(start, x0(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start, x0(2));
  mutable_constraint->AddPointConstraint(start + length, xn(0));
  mutable_constraint->AddPointDerivativeConstraint(start + length, xn(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start + length, xn(2));
  mutable_constraint->AddSecondDerivativeSmoothConstraint();

  if (!obstacle_location_hint.empty()) {
    mutable_constraint->AddBoundary(obstacle_location_hint, lb, ub);
  }

  if (!spline1d_solver.Solve()) {
    LOG(ERROR) << "solve failed";
    return false;
  }

  auto spline = spline1d_solver.spline();
  for (int i = 0; i < s_refs.size(); ++i) {
    result->emplace_back(
        Eigen::Vector3d(spline(s_refs[i]), spline.Derivative(s_refs[i]),
                        spline.SecondOrderDerivative(s_refs[i])));
  }
  return true;
}
}  // namespace planning
