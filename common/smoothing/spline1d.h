

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "common/smoothing/spline1d_seg.h"

namespace common {

class Spline1d {
 public:
  Spline1d() = default;
  Spline1d(const std::vector<double>& x_knots, const uint32_t order);

  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  bool set_splines(const Eigen::MatrixXd& param_matrix, const uint32_t order);

  const std::vector<double>& x_knots() const;
  uint32_t spline_order() const;

  const std::vector<Spline1dSeg>& splines() const;

 private:
  size_t FindSegStartIndex(const double x) const;

 private:
  std::vector<Spline1dSeg> splines_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
};
}  // namespace common
