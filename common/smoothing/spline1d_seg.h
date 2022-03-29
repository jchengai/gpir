

#pragma once

#include <vector>

#include "common/smoothing/polynomialxd.h"

namespace common {

class Spline1dSeg {
 public:
  // order represents the highest order.
  explicit Spline1dSeg(const uint32_t order);
  explicit Spline1dSeg(const std::vector<double>& params);
  ~Spline1dSeg() = default;

  void SetParams(const std::vector<double>& params);
  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  const PolynomialXd& spline_func() const;
  const PolynomialXd& Derivative() const;
  const PolynomialXd& SecondOrderDerivative() const;
  const PolynomialXd& ThirdOrderDerivative() const;

 private:
  inline void SetSplineFunc(const PolynomialXd& spline_func);

  PolynomialXd spline_func_;
  PolynomialXd derivative_;
  PolynomialXd second_order_derivative_;
  PolynomialXd third_order_derivative_;
};
}  // namespace common
