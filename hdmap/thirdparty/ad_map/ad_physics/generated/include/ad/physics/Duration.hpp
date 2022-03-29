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
#define AD_PHYSICS_DURATION_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_DURATION_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_DURATION_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief Forward declaration of DurationSquared
 *
 * Since Duration is defined explicitly as a physical type we have to consider this
 * within operations. Therefore this squared type is defined.
 */
class DurationSquared;

/*!
 * \brief DataType Duration
 *
 * A duration represents a time interval.
 * The unit is: Second
 */
class Duration
{
public:
  /*!
   * \brief constant defining the minimum valid Duration value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Duration value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Duration value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Duration is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Duration()
    : mDuration(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_DURATION_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_DURATION_EXPLICIT_CONVERSION_ Duration(double const iDuration)
    : mDuration(iDuration)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Duration(const Duration &other) = default;

  /*!
   * \brief standard move constructor
   */
  Duration(Duration &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Duration
   *
   * \returns Reference to this Duration.
   */
  Duration &operator=(const Duration &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Duration
   *
   * \returns Reference to this Duration.
   */
  Duration &operator=(Duration &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Duration
   *
   * \returns \c true if both Duration are valid and can be taken as numerically equal
   */
  bool operator==(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mDuration - other.mDuration) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Duration.
   *
   * \returns \c true if one of the Duration is not valid or they can be taken as numerically different
   */
  bool operator!=(const Duration &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Duration.
   *
   * \returns \c true if both Duration are valid and
   *   this Duration is strictly numerically greater than other.
   * \note the precision of Duration is considered
   */
  bool operator>(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mDuration > other.mDuration) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Duration.
   *
   * \returns \c true if both Duration are valid and
   *   this Duration is strictly numerically smaller than other.
   * \note the precision of Duration is considered
   */
  bool operator<(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mDuration < other.mDuration) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Duration.
   *
   * \returns \c true if both Duration are valid and
   *   this Duration is numerically greater than other.
   * \note the precision of Duration is considered
   */
  bool operator>=(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mDuration > other.mDuration) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Duration
   *
   * \returns \c true if both Duration are valid and
   *   this Duration is numerically smaller than other.
   * \note the precision of Duration is considered
   */
  bool operator<=(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mDuration < other.mDuration) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Duration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Duration operator+(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    Duration const result(mDuration + other.mDuration);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Duration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Duration &operator+=(const Duration &other)
  {
    ensureValid();
    other.ensureValid();
    mDuration += other.mDuration;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Duration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Duration operator-(const Duration &other) const
  {
    ensureValid();
    other.ensureValid();
    Duration const result(mDuration - other.mDuration);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Duration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Duration operator-=(const Duration &other)
  {
    ensureValid();
    other.ensureValid();
    mDuration -= other.mDuration;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Duration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   * \note since Duration is a type with physical unit, the multiplication results in the Squared type.
   */
  DurationSquared operator*(const Duration &other) const;

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
  Duration operator*(const double &scalar) const
  {
    ensureValid();
    Duration const result(mDuration * scalar);
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
  Duration operator/(const double &scalar) const
  {
    Duration const scalarDuration(scalar);
    Duration const result(operator/(scalarDuration));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Duration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Duration is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Duration &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mDuration / other.mDuration;
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
  Duration operator-() const
  {
    ensureValid();
    Duration const result(-mDuration);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_DURATION_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_DURATION_EXPLICIT_CONVERSION_ operator double() const
  {
    return mDuration;
  }

  /*!
   * \returns \c true if the Duration in a valid range
   *
   * An Duration value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mDuration);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mDuration)
      && (mDuration <= cMaxValue);
  }

  /*!
   * \brief ensure that the Duration is valid
   *
   * Throws an std::out_of_range() exception if the Duration
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::Duration)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_DURATION_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Duration value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Duration is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Duration
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Duration(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::Duration)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_DURATION_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Duration value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Duration (i.e. \ref cMinValue)
   */
  static Duration getMin()
  {
    return Duration(cMinValue);
  }

  /*!
   * \brief get maximum valid Duration (i.e. \ref cMaxValue)
   */
  static Duration getMax()
  {
    return Duration(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Duration (i.e. \ref cPrecisionValue)
   */
  static Duration getPrecision()
  {
    return Duration(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mDuration;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Duration as double value
 * \param[in] value Duration value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::Duration operator*(const double &other, ::ad::physics::Duration const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Duration
 */
inline ::ad::physics::Duration fabs(const ::ad::physics::Duration other)
{
  ::ad::physics::Duration const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Duration
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Duration>::lowest()  (\see Duration::getMin())
 * std::numeric_limits<Duration>::max()  (\see Duration::getMax())
 * std::numeric_limits<Duration>::epsilon()  (\see Duration::getPrecision())
 */
template <> class numeric_limits<::ad::physics::Duration> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::Duration lowest()
  {
    return ::ad::physics::Duration::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::Duration max()
  {
    return ::ad::physics::Duration::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::Duration epsilon()
  {
    return ::ad::physics::Duration::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DURATION
#define GEN_GUARD_AD_PHYSICS_DURATION
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
 * \param[in] _value Duration value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Duration const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Duration
 */
inline std::string to_string(::ad::physics::Duration const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_DURATION
