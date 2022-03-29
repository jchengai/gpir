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
#define AD_PHYSICS_ANGULARACCELERATION_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_ANGULARACCELERATION_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_ANGULARACCELERATION_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType AngularAcceleration
 *
 * The rate of change of AngularVelocity of an object with respect to time.
 * The unit is: RadianPerSecondSquared
 */
class AngularAcceleration
{
public:
  /*!
   * \brief constant defining the minimum valid AngularAcceleration value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid AngularAcceleration value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed AngularAcceleration value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of AngularAcceleration is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  AngularAcceleration()
    : mAngularAcceleration(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_ANGULARACCELERATION_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is
   * allowed.
   */
  _AD_PHYSICS_ANGULARACCELERATION_EXPLICIT_CONVERSION_ AngularAcceleration(double const iAngularAcceleration)
    : mAngularAcceleration(iAngularAcceleration)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  AngularAcceleration(const AngularAcceleration &other) = default;

  /*!
   * \brief standard move constructor
   */
  AngularAcceleration(AngularAcceleration &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Reference to this AngularAcceleration.
   */
  AngularAcceleration &operator=(const AngularAcceleration &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Reference to this AngularAcceleration.
   */
  AngularAcceleration &operator=(AngularAcceleration &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns \c true if both AngularAcceleration are valid and can be taken as numerically equal
   */
  bool operator==(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mAngularAcceleration - other.mAngularAcceleration) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularAcceleration.
   *
   * \returns \c true if one of the AngularAcceleration is not valid or they can be taken as numerically different
   */
  bool operator!=(const AngularAcceleration &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularAcceleration.
   *
   * \returns \c true if both AngularAcceleration are valid and
   *   this AngularAcceleration is strictly numerically greater than other.
   * \note the precision of AngularAcceleration is considered
   */
  bool operator>(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAngularAcceleration > other.mAngularAcceleration) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularAcceleration.
   *
   * \returns \c true if both AngularAcceleration are valid and
   *   this AngularAcceleration is strictly numerically smaller than other.
   * \note the precision of AngularAcceleration is considered
   */
  bool operator<(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAngularAcceleration < other.mAngularAcceleration) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularAcceleration.
   *
   * \returns \c true if both AngularAcceleration are valid and
   *   this AngularAcceleration is numerically greater than other.
   * \note the precision of AngularAcceleration is considered
   */
  bool operator>=(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAngularAcceleration > other.mAngularAcceleration) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns \c true if both AngularAcceleration are valid and
   *   this AngularAcceleration is numerically smaller than other.
   * \note the precision of AngularAcceleration is considered
   */
  bool operator<=(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAngularAcceleration < other.mAngularAcceleration) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularAcceleration operator+(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    AngularAcceleration const result(mAngularAcceleration + other.mAngularAcceleration);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularAcceleration &operator+=(const AngularAcceleration &other)
  {
    ensureValid();
    other.ensureValid();
    mAngularAcceleration += other.mAngularAcceleration;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularAcceleration operator-(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValid();
    AngularAcceleration const result(mAngularAcceleration - other.mAngularAcceleration);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularAcceleration operator-=(const AngularAcceleration &other)
  {
    ensureValid();
    other.ensureValid();
    mAngularAcceleration -= other.mAngularAcceleration;
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
  AngularAcceleration operator*(const double &scalar) const
  {
    ensureValid();
    AngularAcceleration const result(mAngularAcceleration * scalar);
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
  AngularAcceleration operator/(const double &scalar) const
  {
    AngularAcceleration const scalarAngularAcceleration(scalar);
    AngularAcceleration const result(operator/(scalarAngularAcceleration));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularAcceleration
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since AngularAcceleration is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const AngularAcceleration &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mAngularAcceleration / other.mAngularAcceleration;
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
  AngularAcceleration operator-() const
  {
    ensureValid();
    AngularAcceleration const result(-mAngularAcceleration);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_ANGULARACCELERATION_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_ANGULARACCELERATION_EXPLICIT_CONVERSION_ operator double() const
  {
    return mAngularAcceleration;
  }

  /*!
   * \returns \c true if the AngularAcceleration in a valid range
   *
   * An AngularAcceleration value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mAngularAcceleration);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mAngularAcceleration)
      && (mAngularAcceleration <= cMaxValue);
  }

  /*!
   * \brief ensure that the AngularAcceleration is valid
   *
   * Throws an std::out_of_range() exception if the AngularAcceleration
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::AngularAcceleration)>> {} value out of range",
                   *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_ANGULARACCELERATION_THROWS_EXCEPTION == 1)
      throw std::out_of_range("AngularAcceleration value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the AngularAcceleration is valid and non zero
   *
   * Throws an std::out_of_range() exception if the AngularAcceleration
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(AngularAcceleration(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::AngularAcceleration)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_ANGULARACCELERATION_THROWS_EXCEPTION == 1)
      throw std::out_of_range("AngularAcceleration value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid AngularAcceleration (i.e. \ref cMinValue)
   */
  static AngularAcceleration getMin()
  {
    return AngularAcceleration(cMinValue);
  }

  /*!
   * \brief get maximum valid AngularAcceleration (i.e. \ref cMaxValue)
   */
  static AngularAcceleration getMax()
  {
    return AngularAcceleration(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of AngularAcceleration (i.e. \ref cPrecisionValue)
   */
  static AngularAcceleration getPrecision()
  {
    return AngularAcceleration(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mAngularAcceleration;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other AngularAcceleration as double value
 * \param[in] value AngularAcceleration value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::AngularAcceleration operator*(const double &other,
                                                    ::ad::physics::AngularAcceleration const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for AngularAcceleration
 */
inline ::ad::physics::AngularAcceleration fabs(const ::ad::physics::AngularAcceleration other)
{
  ::ad::physics::AngularAcceleration const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for AngularAcceleration
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<AngularAcceleration>::lowest()  (\see AngularAcceleration::getMin())
 * std::numeric_limits<AngularAcceleration>::max()  (\see AngularAcceleration::getMax())
 * std::numeric_limits<AngularAcceleration>::epsilon()  (\see AngularAcceleration::getPrecision())
 */
template <> class numeric_limits<::ad::physics::AngularAcceleration> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::AngularAcceleration lowest()
  {
    return ::ad::physics::AngularAcceleration::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::AngularAcceleration max()
  {
    return ::ad::physics::AngularAcceleration::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::AngularAcceleration epsilon()
  {
    return ::ad::physics::AngularAcceleration::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ANGULARACCELERATION
#define GEN_GUARD_AD_PHYSICS_ANGULARACCELERATION
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
 * \param[in] _value AngularAcceleration value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, AngularAcceleration const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for AngularAcceleration
 */
inline std::string to_string(::ad::physics::AngularAcceleration const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ANGULARACCELERATION
