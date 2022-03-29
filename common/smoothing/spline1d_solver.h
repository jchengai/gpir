

#pragma once

#include <Eigen/Core>
#include <vector>

#include "common/smoothing/spline1d.h"
#include "common/smoothing/spline1d_constraint.h"
#include "common/smoothing/spline1d_kernel.h"

namespace common {

class Spline1dSolver {
 public:
  Spline1dSolver(const std::vector<double>& x_knots, const uint32_t order)
      : spline_(x_knots, order),
        constraint_(x_knots, order),
        kernel_(x_knots, order) {}

  virtual ~Spline1dSolver() = default;

  virtual void Reset(const std::vector<double>& t_knots,
                     const uint32_t order) = 0;

  // customize setup
  virtual Spline1dConstraint* mutable_constraint() = 0;
  virtual Spline1dKernel* mutable_kernel() = 0;
  virtual Spline1d* mutable_spline() = 0;

  // solve
  virtual bool Solve() = 0;

  // extract
  virtual const Spline1d& spline() const = 0;

 protected:
  Spline1d spline_;
  Spline1dConstraint constraint_;
  Spline1dKernel kernel_;
};
}  // namespace common
