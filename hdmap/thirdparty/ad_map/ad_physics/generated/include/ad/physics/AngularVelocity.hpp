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
#define AD_PHYSICS_ANGULARVELOCITY_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_ANGULARVELOCITY_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_ANGULARVELOCITY_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType AngularVelocity
 *
 * The rate of change of angular velocity of an object with respect to time.
 * The unit is: RadianPerSecond
 */
class AngularVelocity
{
public:
  /*!
   * \brief constant defining the minimum valid AngularVelocity value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid AngularVelocity value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed AngularVelocity value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of AngularVelocity is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  AngularVelocity()
    : mAngularVelocity(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_ANGULARVELOCITY_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_ANGULARVELOCITY_EXPLICIT_CONVERSION_ AngularVelocity(double const iAngularVelocity)
    : mAngularVelocity(iAngularVelocity)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  AngularVelocity(const AngularVelocity &other) = default;

  /*!
   * \brief standard move constructor
   */
  AngularVelocity(AngularVelocity &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Reference to this AngularVelocity.
   */
  AngularVelocity &operator=(const AngularVelocity &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Reference to this AngularVelocity.
   */
  AngularVelocity &operator=(AngularVelocity &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns \c true if both AngularVelocity are valid and can be taken as numerically equal
   */
  bool operator==(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mAngularVelocity - other.mAngularVelocity) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity.
   *
   * \returns \c true if one of the AngularVelocity is not valid or they can be taken as numerically different
   */
  bool operator!=(const AngularVelocity &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity.
   *
   * \returns \c true if both AngularVelocity are valid and
   *   this AngularVelocity is strictly numerically greater than other.
   * \note the precision of AngularVelocity is considered
   */
  bool operator>(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAngularVelocity > other.mAngularVelocity) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity.
   *
   * \returns \c true if both AngularVelocity are valid and
   *   this AngularVelocity is strictly numerically smaller than other.
   * \note the precision of AngularVelocity is considered
   */
  bool operator<(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAngularVelocity < other.mAngularVelocity) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity.
   *
   * \returns \c true if both AngularVelocity are valid and
   *   this AngularVelocity is numerically greater than other.
   * \note the precision of AngularVelocity is considered
   */
  bool operator>=(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAngularVelocity > other.mAngularVelocity) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns \c true if both AngularVelocity are valid and
   *   this AngularVelocity is numerically smaller than other.
   * \note the precision of AngularVelocity is considered
   */
  bool operator<=(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAngularVelocity < other.mAngularVelocity) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularVelocity operator+(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    AngularVelocity const result(mAngularVelocity + other.mAngularVelocity);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularVelocity &operator+=(const AngularVelocity &other)
  {
    ensureValid();
    other.ensureValid();
    mAngularVelocity += other.mAngularVelocity;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularVelocity operator-(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValid();
    AngularVelocity const result(mAngularVelocity - other.mAngularVelocity);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  AngularVelocity operator-=(const AngularVelocity &other)
  {
    ensureValid();
    other.ensureValid();
    mAngularVelocity -= other.mAngularVelocity;
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
  AngularVelocity operator*(const double &scalar) const
  {
    ensureValid();
    AngularVelocity const result(mAngularVelocity * scalar);
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
  AngularVelocity operator/(const double &scalar) const
  {
    AngularVelocity const scalarAngularVelocity(scalar);
    AngularVelocity const result(operator/(scalarAngularVelocity));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other AngularVelocity
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since AngularVelocity is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const AngularVelocity &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mAngularVelocity / other.mAngularVelocity;
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
  AngularVelocity operator-() const
  {
    ensureValid();
    AngularVelocity const result(-mAngularVelocity);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_ANGULARVELOCITY_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_ANGULARVELOCITY_EXPLICIT_CONVERSION_ operator double() const
  {
    return mAngularVelocity;
  }

  /*!
   * \returns \c true if the AngularVelocity in a valid range
   *
   * An AngularVelocity value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mAngularVelocity);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mAngularVelocity)
      && (mAngularVelocity <= cMaxValue);
  }

  /*!
   * \brief ensure that the AngularVelocity is valid
   *
   * Throws an std::out_of_range() exception if the AngularVelocity
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::AngularVelocity)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_ANGULARVELOCITY_THROWS_EXCEPTION == 1)
      throw std::out_of_range("AngularVelocity value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the AngularVelocity is valid and non zero
   *
   * Throws an std::out_of_range() exception if the AngularVelocity
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(AngularVelocity(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::AngularVelocity)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_ANGULARVELOCITY_THROWS_EXCEPTION == 1)
      throw std::out_of_range("AngularVelocity value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid AngularVelocity (i.e. \ref cMinValue)
   */
  static AngularVelocity getMin()
  {
    return AngularVelocity(cMinValue);
  }

  /*!
   * \brief get maximum valid AngularVelocity (i.e. \ref cMaxValue)
   */
  static AngularVelocity getMax()
  {
    return AngularVelocity(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of AngularVelocity (i.e. \ref cPrecisionValue)
   */
  static AngularVelocity getPrecision()
  {
    return AngularVelocity(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mAngularVelocity;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other AngularVelocity as double value
 * \param[in] value AngularVelocity value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::AngularVelocity operator*(const double &other, ::ad::physics::AngularVelocity const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for AngularVelocity
 */
inline ::ad::physics::AngularVelocity fabs(const ::ad::physics::AngularVelocity other)
{
  ::ad::physics::AngularVelocity const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for AngularVelocity
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<AngularVelocity>::lowest()  (\see AngularVelocity::getMin())
 * std::numeric_limits<AngularVelocity>::max()  (\see AngularVelocity::getMax())
 * std::numeric_limits<AngularVelocity>::epsilon()  (\see AngularVelocity::getPrecision())
 */
template <> class numeric_limits<::ad::physics::AngularVelocity> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::AngularVelocity lowest()
  {
    return ::ad::physics::AngularVelocity::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::AngularVelocity max()
  {
    return ::ad::physics::AngularVelocity::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::AngularVelocity epsilon()
  {
    return ::ad::physics::AngularVelocity::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ANGULARVELOCITY
#define GEN_GUARD_AD_PHYSICS_ANGULARVELOCITY
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
 * \param[in] _value AngularVelocity value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, AngularVelocity const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for AngularVelocity
 */
inline std::string to_string(::ad::physics::AngularVelocity const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ANGULARVELOCITY
