

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cinttypes>
#include <vector>

#include "common/smoothing/spline1d_kernel.h"

namespace common {
class Spline2dKernel {
 public:
  Spline2dKernel(const std::vector<double>& t_knots,
                 const uint32_t spline_order);

  void AddRegularization(const double regularized_param);
  void Add2dDerivativeKernelMatrix(const double weight);
  void Add2dSecondOrderDerivativeMatrix(const double weight);
  void Add2dThirdOrderDerivativeMatrix(const double weight);

  bool Add2dReferenceLineKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft, const double weight);

  void Add2dLateralOffsetKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft,
      const ::std::vector<double> ref_angle, const double weight);

  void Add2dLongitudinalOffsetKernelMatrix(
      const std::vector<double>& t_coord,
      const vector_Eigen<Eigen::Vector2d>& ref_ft,
      const ::std::vector<double> ref_angle, const double weight);

  size_t FindSegStartIndex(const double t) const;

  const Eigen::MatrixXd& kernel_matrix();
  const Eigen::MatrixXd& gradient();

 private:
  /* kernel only contains x's component */
  Spline1dKernel x_kernel_;
  /* kernel only contains y's component */
  Spline1dKernel y_kernel_;
  /* kernel contains both x and y's component */
  Eigen::MatrixXd cooperative_kernel_;

  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd gradient_;

  std::vector<double> t_knots_;

  uint32_t spline_order_ = 0;
  uint32_t spline_param_num_ = 0;
  uint32_t total_params_ = 0;
};
}  // namespace common
