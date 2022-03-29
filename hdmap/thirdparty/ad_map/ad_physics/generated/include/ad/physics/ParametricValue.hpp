/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/**
 * Generated file
 * @file
 *
 * Generator Version : 11.0.0-1997
 */

#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief Define to indicate whether throwing exceptions is enabled
 */
#define AD_PHYSICS_PARAMETRICVALUE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "only explicit
 * conversion".
 */
#define _AD_PHYSICS_PARAMETRICVALUE_EXPLICIT_CONVERSION_ explicit
#else
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "implicit
 * conversion allowed".
 */
#define _AD_PHYSICS_PARAMETRICVALUE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType ParametricValue
 *
 * A parametric value is a ratio in the range of [0.0; 1.0] describing the
 * relative progress. The unit is: Ratio
 */
class ParametricValue {
 public:
  /*!
   * \brief constant defining the minimum valid ParametricValue value (used in
   * isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid ParametricValue value (used in
   * isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed ParametricValue value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of ParametricValue is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  ParametricValue()
      : mParametricValue(std::numeric_limits<double>::quiet_NaN()) {}

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_PARAMETRICVALUE_EXPLICIT_CONVERSION\_ defines, if
   * only an explicit conversion is allowed.
   */
  _AD_PHYSICS_PARAMETRICVALUE_EXPLICIT_CONVERSION_ ParametricValue(
      double const iParametricValue)
      : mParametricValue(iParametricValue) {}

  /*!
   * \brief standard copy constructor
   */
  ParametricValue(const ParametricValue &other) = default;

  /*!
   * \brief standard move constructor
   */
  ParametricValue(ParametricValue &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Reference to this ParametricValue.
   */
  ParametricValue &operator=(const ParametricValue &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Reference to this ParametricValue.
   */
  ParametricValue &operator=(ParametricValue &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns \c true if both ParametricValue are valid and can be taken as
   * numerically equal
   */
  bool operator==(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    return std::fabs(mParametricValue - other.mParametricValue) <
           cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricValue.
   *
   * \returns \c true if one of the ParametricValue is not valid or they can be
   * taken as numerically different
   */
  bool operator!=(const ParametricValue &other) const {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricValue.
   *
   * \returns \c true if both ParametricValue are valid and
   *   this ParametricValue is strictly numerically greater than other.
   * \note the precision of ParametricValue is considered
   */
  bool operator>(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    return (mParametricValue > other.mParametricValue) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricValue.
   *
   * \returns \c true if both ParametricValue are valid and
   *   this ParametricValue is strictly numerically smaller than other.
   * \note the precision of ParametricValue is considered
   */
  bool operator<(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    return (mParametricValue < other.mParametricValue) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricValue.
   *
   * \returns \c true if both ParametricValue are valid and
   *   this ParametricValue is numerically greater than other.
   * \note the precision of ParametricValue is considered
   */
  bool operator>=(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    return ((mParametricValue > other.mParametricValue) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns \c true if both ParametricValue are valid and
   *   this ParametricValue is numerically smaller than other.
   * \note the precision of ParametricValue is considered
   */
  bool operator<=(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    return ((mParametricValue < other.mParametricValue) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ParametricValue operator+(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    ParametricValue const result(mParametricValue + other.mParametricValue);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ParametricValue &operator+=(const ParametricValue &other) {
    ensureValid();
    other.ensureValid();
    mParametricValue += other.mParametricValue;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ParametricValue operator-(const ParametricValue &other) const {
    ensureValid();
    other.ensureValid();
    ParametricValue const result(mParametricValue - other.mParametricValue);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ParametricValue operator-=(const ParametricValue &other) {
    ensureValid();
    other.ensureValid();
    mParametricValue -= other.mParametricValue;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] scalar Scalar double value
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if \c value or the result of
   *   the operation is not valid
   */
  ParametricValue operator*(const double &scalar) const {
    ensureValid();
    ParametricValue const result(mParametricValue * scalar);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] scalar Scalar double value
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if this or the result of
   *   the operation is not valid or other is zero
   */
  ParametricValue operator/(const double &scalar) const {
    ParametricValue const scalarParametricValue(scalar);
    ParametricValue const result(operator/(scalarParametricValue));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ParametricValue
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid or other is zero \note since
   * ParametricValue is a type with physical unit, the division results in the
   * dimensionless type.
   */
  double operator/(const ParametricValue &other) const {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mParametricValue / other.mParametricValue;
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if this or the result of
   *   the operation is not valid
   */
  ParametricValue operator-() const {
    ensureValid();
    ParametricValue const result(-mParametricValue);
    result.ensureValid();  // LCOV_EXCL_BR_LINE Some types do not throw an
                           // exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_PARAMETRICVALUE_EXPLICIT_CONVERSION\_ defines, if
   * only explicit calls are allowed.
   */
  _AD_PHYSICS_PARAMETRICVALUE_EXPLICIT_CONVERSION_ operator double() const {
    return mParametricValue;
  }

  /*!
   * \returns \c true if the ParametricValue in a valid range
   *
   * An ParametricValue value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const {
    auto const valueClass = std::fpclassify(mParametricValue);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) &&
           (cMinValue <= mParametricValue) && (mParametricValue <= cMaxValue);
  }

  /*!
   * \brief ensure that the ParametricValue is valid
   *
   * Throws an std::out_of_range() exception if the ParametricValue
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const {
    if (!isValid()) {
      spdlog::info(
          "ensureValid(::ad::physics::ParametricValue)>> {} value out of range",
          *this);  // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_PARAMETRICVALUE_THROWS_EXCEPTION == 1)
      throw std::out_of_range(
          "ParametricValue value out of range");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the ParametricValue is valid and non zero
   *
   * Throws an std::out_of_range() exception if the ParametricValue
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const {
    ensureValid();
    if (operator==(ParametricValue(0.)))  // LCOV_EXCL_BR_LINE
    {
      spdlog::info(
          "ensureValid(::ad::physics::ParametricValue)>> {} value is zero",
          *this);  // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_PARAMETRICVALUE_THROWS_EXCEPTION == 1)
      throw std::out_of_range(
          "ParametricValue value is zero");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid ParametricValue (i.e. \ref cMinValue)
   */
  static ParametricValue getMin() { return ParametricValue(cMinValue); }

  /*!
   * \brief get maximum valid ParametricValue (i.e. \ref cMaxValue)
   */
  static ParametricValue getMax() { return ParametricValue(cMaxValue); }

  /*!
   * \brief get assumed accuracy of ParametricValue (i.e. \ref cPrecisionValue)
   */
  static ParametricValue getPrecision() {
    return ParametricValue(cPrecisionValue);
  }

 private:
  /*!
   * \brief the actual value of the type
   */
  double mParametricValue;
};

}  // namespace physics
}  // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other ParametricValue as double value
 * \param[in] value ParametricValue value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::ParametricValue operator*(
    const double &other, ::ad::physics::ParametricValue const &value) {
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for ParametricValue
 */
inline ::ad::physics::ParametricValue fabs(
    const ::ad::physics::ParametricValue other) {
  ::ad::physics::ParametricValue const result(
      std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for ParametricValue
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<ParametricValue>::lowest()  (\see
 * ParametricValue::getMin()) std::numeric_limits<ParametricValue>::max()  (\see
 * ParametricValue::getMax()) std::numeric_limits<ParametricValue>::epsilon()
 * (\see ParametricValue::getPrecision())
 */
template <>
class numeric_limits<::ad::physics::ParametricValue>
    : public numeric_limits<double> {
 public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::ParametricValue lowest() {
    return ::ad::physics::ParametricValue::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::ParametricValue max() {
    return ::ad::physics::ParametricValue::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::ParametricValue epsilon() {
    return ::ad::physics::ParametricValue::getPrecision();
  }
};

}  // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_PARAMETRICVALUE
#define GEN_GUARD_AD_PHYSICS_PARAMETRICVALUE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value ParametricValue value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os,
                                ParametricValue const &_value) {
  return os << double(_value);
}

}  // namespace physics
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ParametricValue
 */
inline std::string to_string(::ad::physics::ParametricValue const &value) {
  return to_string(static_cast<double>(value));
}
}  // namespace std
#endif  // GEN_GUARD_AD_PHYSICS_PARAMETRICVALUE
