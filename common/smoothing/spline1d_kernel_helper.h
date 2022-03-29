

#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>

namespace common {

class Spline1dKernelHelper {
 public:
  static Spline1dKernelHelper& Instance();

  Eigen::MatrixXd Kernel(const uint32_t spline_order,
                         const uint32_t nth_derivative,
                         const double integral_length);

 private:
  Spline1dKernelHelper();

  Spline1dKernelHelper(const Spline1dKernelHelper&) = delete;

  Spline1dKernelHelper& operator=(const Spline1dKernelHelper&) = delete;

  Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                   const double accumulated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                              const double accumulated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                             const double accumulated_x);

  void IntegratedTermMatrix(const uint32_t num_of_params,
                            const uint16_t derivative, const double x,
                            Eigen::MatrixXd* term_matrix) const;

  void CalculateFx(const uint32_t num_of_params);
  void CalculateDerivative(const uint32_t num_of_params);
  void CalculateSecondOrderDerivative(const uint32_t num_of_params);
  void CalculateThirdOrderDerivative(const uint32_t num_of_params);

  void BuildKernelMap();

 private:
  std::unordered_map<std::string, Eigen::MatrixXd> kernel_map_;

  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;
};
}  // namespace common
