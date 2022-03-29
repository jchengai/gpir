

#pragma once

#include <vector>

#include "common/smoothing/spline1d_solver.h"

namespace common {

class OsqpSpline1dSolver final : public Spline1dSolver {
 public:
  OsqpSpline1dSolver(const std::vector<double>& t_knots, const uint32_t order);

    // ~OsqpSpline1dSolver() = default;

  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline1dConstraint* mutable_constraint() override;
  Spline1dKernel* mutable_kernel() override;
  Spline1d* mutable_spline() override;

  // solve
  bool Solve() override;

  // extract
  const Spline1d& spline() const override;
};

}  // namespace common
