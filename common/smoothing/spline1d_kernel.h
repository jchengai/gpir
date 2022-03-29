

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cinttypes>
#include <vector>

#include "common/base/type.h"

namespace common {

class Spline1dKernel {
 public:
  Spline1dKernel() = default;
  Spline1dKernel(const std::vector<double>& x_knots,
                 const uint32_t spline_order);

  void AddKernel(const Eigen::MatrixXd& kernel);
  void AddGradient(const Eigen::MatrixXd& gradient);

  void AddRegularization(const double regularized_param);

  void AddDerivativeKernelMatrix(const double weight);
  void AddSecondOrderDerivativeMatrix(const double weight);
  void AddThirdOrderDerivativeMatrix(const double weight);

  bool AddReferenceLineKernelMatrix(const std::vector<double>& x_coord,
                                    const std::vector<double>& ref_fx,
                                    const double weight);

  bool AddDerivativeReferenceLineKernelMatrix(
      const std::vector<double>& x_coord, const std::vector<double>& ref_dfx,
      const double weight);

  const Eigen::MatrixXd& kernel_matrix() const;
  const Eigen::MatrixXd& gradient() const;

 private:
  void AddNthDerivativekernelMatrix(const uint32_t n, const double weight);

  size_t FindSegStartIndex(const double x) const;

 private:
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd gradient_;

  std::vector<double> x_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t total_params_ = 0;
};
}  // namespace common
