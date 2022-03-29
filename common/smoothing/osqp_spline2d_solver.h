

#pragma once

#include <vector>

#include "common/smoothing/spline2d_solver.h"

namespace common {

class OsqpSpline2dSolver final : public Spline2dSolver {
 public:
  OsqpSpline2dSolver(const std::vector<double>& t_knots, const uint32_t order);

  ~OsqpSpline2dSolver() = default;

  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline2dConstraint* mutable_constraint() override;
  Spline2dKernel* mutable_kernel() override;
  Spline2d* mutable_spline() override;

  // solve
  bool Solve() override;

  // extract
  const Spline2d& spline() const override;
};
}  // namespace common
