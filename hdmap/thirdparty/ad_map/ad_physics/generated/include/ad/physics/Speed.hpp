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
#define AD_PHYSICS_SPEED_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_SPEED_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_SPEED_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief Forward declaration of SpeedSquared
 *
 * Since Speed is defined explicitly as a physical type we have to consider this
 * within operations. Therefore this squared type is defined.
 */
class SpeedSquared;

/*!
 * \brief DataType Speed
 *
 * The rate of change of an object's position with respect to time. The speed of an object is the magnitude of its
 * velocity.
 * The unit is: MeterPerSecond
 */
class Speed
{
public:
  /*!
   * \brief constant defining the minimum valid Speed value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Speed value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Speed value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Speed is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Speed()
    : mSpeed(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_SPEED_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_SPEED_EXPLICIT_CONVERSION_ Speed(double const iSpeed)
    : mSpeed(iSpeed)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Speed(const Speed &other) = default;

  /*!
   * \brief standard move constructor
   */
  Speed(Speed &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Speed
   *
   * \returns Reference to this Speed.
   */
  Speed &operator=(const Speed &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Speed
   *
   * \returns Reference to this Speed.
   */
  Speed &operator=(Speed &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Speed
   *
   * \returns \c true if both Speed are valid and can be taken as numerically equal
   */
  bool operator==(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mSpeed - other.mSpeed) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Speed.
   *
   * \returns \c true if one of the Speed is not valid or they can be taken as numerically different
   */
  bool operator!=(const Speed &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Speed.
   *
   * \returns \c true if both Speed are valid and
   *   this Speed is strictly numerically greater than other.
   * \note the precision of Speed is considered
   */
  bool operator>(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mSpeed > other.mSpeed) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Speed.
   *
   * \returns \c true if both Speed are valid and
   *   this Speed is strictly numerically smaller than other.
   * \note the precision of Speed is considered
   */
  bool operator<(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mSpeed < other.mSpeed) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Speed.
   *
   * \returns \c true if both Speed are valid and
   *   this Speed is numerically greater than other.
   * \note the precision of Speed is considered
   */
  bool operator>=(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mSpeed > other.mSpeed) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Speed
   *
   * \returns \c true if both Speed are valid and
   *   this Speed is numerically smaller than other.
   * \note the precision of Speed is considered
   */
  bool operator<=(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mSpeed < other.mSpeed) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Speed
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Speed operator+(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    Speed const result(mSpeed + other.mSpeed);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Speed
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Speed &operator+=(const Speed &other)
  {
    ensureValid();
    other.ensureValid();
    mSpeed += other.mSpeed;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Speed
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Speed operator-(const Speed &other) const
  {
    ensureValid();
    other.ensureValid();
    Speed const result(mSpeed - other.mSpeed);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Speed
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Speed operator-=(const Speed &other)
  {
    ensureValid();
    other.ensureValid();
    mSpeed -= other.mSpeed;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Speed
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   * \note since Speed is a type with physical unit, the multiplication results in the Squared type.
   */
  SpeedSquared operator*(const Speed &other) const;

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
  Speed operator*(const double &scalar) const
  {
    ensureValid();
    Speed const result(mSpeed * scalar);
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
  Speed operator/(const double &scalar) const
  {
    Speed const scalarSpeed(scalar);
    Speed const result(operator/(scalarSpeed));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Speed
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Speed is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Speed &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mSpeed / other.mSpeed;
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
  Speed operator-() const
  {
    ensureValid();
    Speed const result(-mSpeed);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_SPEED_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_SPEED_EXPLICIT_CONVERSION_ operator double() const
  {
    return mSpeed;
  }

  /*!
   * \returns \c true if the Speed in a valid range
   *
   * An Speed value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mSpeed);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mSpeed) && (mSpeed <= cMaxValue);
  }

  /*!
   * \brief ensure that the Speed is valid
   *
   * Throws an std::out_of_range() exception if the Speed
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::Speed)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_SPEED_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Speed value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Speed is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Speed
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Speed(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::Speed)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_SPEED_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Speed value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Speed (i.e. \ref cMinValue)
   */
  static Speed getMin()
  {
    return Speed(cMinValue);
  }

  /*!
   * \brief get maximum valid Speed (i.e. \ref cMaxValue)
   */
  static Speed getMax()
  {
    return Speed(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Speed (i.e. \ref cPrecisionValue)
   */
  static Speed getPrecision()
  {
    return Speed(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mSpeed;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Speed as double value
 * \param[in] value Speed value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::Speed operator*(const double &other, ::ad::physics::Speed const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Speed
 */
inline ::ad::physics::Speed fabs(const ::ad::physics::Speed other)
{
  ::ad::physics::Speed const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Speed
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Speed>::lowest()  (\see Speed::getMin())
 * std::numeric_limits<Speed>::max()  (\see Speed::getMax())
 * std::numeric_limits<Speed>::epsilon()  (\see Speed::getPrecision())
 */
template <> class numeric_limits<::ad::physics::Speed> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::Speed lowest()
  {
    return ::ad::physics::Speed::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::Speed max()
  {
    return ::ad::physics::Speed::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::Speed epsilon()
  {
    return ::ad::physics::Speed::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_SPEED
#define GEN_GUARD_AD_PHYSICS_SPEED
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
 * \param[in] _value Speed value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Speed const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Speed
 */
inline std::string to_string(::ad::physics::Speed const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_SPEED
