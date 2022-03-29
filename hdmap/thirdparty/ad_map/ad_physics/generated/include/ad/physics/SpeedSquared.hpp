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
#define AD_PHYSICS_SPEEDSQUARED_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_SPEEDSQUARED_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_SPEEDSQUARED_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief Forward declaration of Speed
 *
 * Since SpeedSquared is defined explicitly as a physical type we have to consider this
 * within operations. Therefore this square-rooted type is defined.
 */
class Speed;

/*!
 * \brief DataType SpeedSquared
 *
 * SpeedSquared represents a squared Speed.
 * The unit is: SquareMeterPerSecondSquared
 */
class SpeedSquared
{
public:
  /*!
   * \brief constant defining the minimum valid SpeedSquared value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid SpeedSquared value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed SpeedSquared value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of SpeedSquared is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  SpeedSquared()
    : mSpeedSquared(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_SPEEDSQUARED_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_SPEEDSQUARED_EXPLICIT_CONVERSION_ SpeedSquared(double const iSpeedSquared)
    : mSpeedSquared(iSpeedSquared)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  SpeedSquared(const SpeedSquared &other) = default;

  /*!
   * \brief standard move constructor
   */
  SpeedSquared(SpeedSquared &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Reference to this SpeedSquared.
   */
  SpeedSquared &operator=(const SpeedSquared &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Reference to this SpeedSquared.
   */
  SpeedSquared &operator=(SpeedSquared &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns \c true if both SpeedSquared are valid and can be taken as numerically equal
   */
  bool operator==(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mSpeedSquared - other.mSpeedSquared) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedSquared.
   *
   * \returns \c true if one of the SpeedSquared is not valid or they can be taken as numerically different
   */
  bool operator!=(const SpeedSquared &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedSquared.
   *
   * \returns \c true if both SpeedSquared are valid and
   *   this SpeedSquared is strictly numerically greater than other.
   * \note the precision of SpeedSquared is considered
   */
  bool operator>(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mSpeedSquared > other.mSpeedSquared) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedSquared.
   *
   * \returns \c true if both SpeedSquared are valid and
   *   this SpeedSquared is strictly numerically smaller than other.
   * \note the precision of SpeedSquared is considered
   */
  bool operator<(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mSpeedSquared < other.mSpeedSquared) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedSquared.
   *
   * \returns \c true if both SpeedSquared are valid and
   *   this SpeedSquared is numerically greater than other.
   * \note the precision of SpeedSquared is considered
   */
  bool operator>=(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mSpeedSquared > other.mSpeedSquared) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns \c true if both SpeedSquared are valid and
   *   this SpeedSquared is numerically smaller than other.
   * \note the precision of SpeedSquared is considered
   */
  bool operator<=(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mSpeedSquared < other.mSpeedSquared) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  SpeedSquared operator+(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    SpeedSquared const result(mSpeedSquared + other.mSpeedSquared);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  SpeedSquared &operator+=(const SpeedSquared &other)
  {
    ensureValid();
    other.ensureValid();
    mSpeedSquared += other.mSpeedSquared;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  SpeedSquared operator-(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    SpeedSquared const result(mSpeedSquared - other.mSpeedSquared);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  SpeedSquared operator-=(const SpeedSquared &other)
  {
    ensureValid();
    other.ensureValid();
    mSpeedSquared -= other.mSpeedSquared;
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
  SpeedSquared operator*(const double &scalar) const
  {
    ensureValid();
    SpeedSquared const result(mSpeedSquared * scalar);
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
  SpeedSquared operator/(const double &scalar) const
  {
    SpeedSquared const scalarSpeedSquared(scalar);
    SpeedSquared const result(operator/(scalarSpeedSquared));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other SpeedSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since SpeedSquared is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const SpeedSquared &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mSpeedSquared / other.mSpeedSquared;
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
  SpeedSquared operator-() const
  {
    ensureValid();
    SpeedSquared const result(-mSpeedSquared);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_SPEEDSQUARED_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_SPEEDSQUARED_EXPLICIT_CONVERSION_ operator double() const
  {
    return mSpeedSquared;
  }

  /*!
   * \returns \c true if the SpeedSquared in a valid range
   *
   * An SpeedSquared value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mSpeedSquared);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mSpeedSquared)
      && (mSpeedSquared <= cMaxValue);
  }

  /*!
   * \brief ensure that the SpeedSquared is valid
   *
   * Throws an std::out_of_range() exception if the SpeedSquared
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::SpeedSquared)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_SPEEDSQUARED_THROWS_EXCEPTION == 1)
      throw std::out_of_range("SpeedSquared value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the SpeedSquared is valid and non zero
   *
   * Throws an std::out_of_range() exception if the SpeedSquared
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(SpeedSquared(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::SpeedSquared)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_SPEEDSQUARED_THROWS_EXCEPTION == 1)
      throw std::out_of_range("SpeedSquared value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid SpeedSquared (i.e. \ref cMinValue)
   */
  static SpeedSquared getMin()
  {
    return SpeedSquared(cMinValue);
  }

  /*!
   * \brief get maximum valid SpeedSquared (i.e. \ref cMaxValue)
   */
  static SpeedSquared getMax()
  {
    return SpeedSquared(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of SpeedSquared (i.e. \ref cPrecisionValue)
   */
  static SpeedSquared getPrecision()
  {
    return SpeedSquared(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mSpeedSquared;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other SpeedSquared as double value
 * \param[in] value SpeedSquared value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::SpeedSquared operator*(const double &other, ::ad::physics::SpeedSquared const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for SpeedSquared
 */
inline ::ad::physics::SpeedSquared fabs(const ::ad::physics::SpeedSquared other)
{
  ::ad::physics::SpeedSquared const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for SpeedSquared
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<SpeedSquared>::lowest()  (\see SpeedSquared::getMin())
 * std::numeric_limits<SpeedSquared>::max()  (\see SpeedSquared::getMax())
 * std::numeric_limits<SpeedSquared>::epsilon()  (\see SpeedSquared::getPrecision())
 */
template <> class numeric_limits<::ad::physics::SpeedSquared> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::SpeedSquared lowest()
  {
    return ::ad::physics::SpeedSquared::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::SpeedSquared max()
  {
    return ::ad::physics::SpeedSquared::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::SpeedSquared epsilon()
  {
    return ::ad::physics::SpeedSquared::getPrecision();
  }
};

/*!
 * \brief overload of the std::sqrt for SpeedSquared
 *
 * The square root of a squared type is basic type: Speed.
 */
::ad::physics::Speed sqrt(::ad::physics::SpeedSquared const other);

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_SPEEDSQUARED
#define GEN_GUARD_AD_PHYSICS_SPEEDSQUARED
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
 * \param[in] _value SpeedSquared value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, SpeedSquared const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for SpeedSquared
 */
inline std::string to_string(::ad::physics::SpeedSquared const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_SPEEDSQUARED
