

#pragma once

#include <cinttypes>
#include <vector>

namespace common {

/**
 * @class PolynomialXd
 * @brief y = a0 + a1*x + a2*x2^2 + ... + an*xn^n
 */
class PolynomialXd {
 public:
  PolynomialXd() = default;
  explicit PolynomialXd(const std::uint32_t order);
  explicit PolynomialXd(const std::vector<double>& params);

  ~PolynomialXd() = default;

  double operator()(const double x) const;
  double operator[](const std::uint32_t index) const;

  void SetParams(const std::vector<double>& params);

  static PolynomialXd DerivedFrom(const PolynomialXd& base);
  static PolynomialXd IntegratedFrom(const PolynomialXd& base,
                                     const double intercept = 0.0);

  std::uint32_t order() const;

  const std::vector<double>& params() const;

 private:
  std::vector<double> params_;
};
}  // namespace common
