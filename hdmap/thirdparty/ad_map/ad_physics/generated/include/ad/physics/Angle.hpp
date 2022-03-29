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
#define AD_PHYSICS_ANGLE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_ANGLE_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_ANGLE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType Angle
 *
 * An angle measured in radians
 * The unit is: Radian
 */
class Angle
{
public:
  /*!
   * \brief constant defining the minimum valid Angle value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Angle value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Angle value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Angle is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Angle()
    : mAngle(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_ANGLE_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_ANGLE_EXPLICIT_CONVERSION_ Angle(double const iAngle)
    : mAngle(iAngle)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Angle(const Angle &other) = default;

  /*!
   * \brief standard move constructor
   */
  Angle(Angle &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Angle
   *
   * \returns Reference to this Angle.
   */
  Angle &operator=(const Angle &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Angle
   *
   * \returns Reference to this Angle.
   */
  Angle &operator=(Angle &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Angle
   *
   * \returns \c true if both Angle are valid and can be taken as numerically equal
   */
  bool operator==(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mAngle - other.mAngle) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Angle.
   *
   * \returns \c true if one of the Angle is not valid or they can be taken as numerically different
   */
  bool operator!=(const Angle &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Angle.
   *
   * \returns \c true if both Angle are valid and
   *   this Angle is strictly numerically greater than other.
   * \note the precision of Angle is considered
   */
  bool operator>(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAngle > other.mAngle) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Angle.
   *
   * \returns \c true if both Angle are valid and
   *   this Angle is strictly numerically smaller than other.
   * \note the precision of Angle is considered
   */
  bool operator<(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAngle < other.mAngle) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Angle.
   *
   * \returns \c true if both Angle are valid and
   *   this Angle is numerically greater than other.
   * \note the precision of Angle is considered
   */
  bool operator>=(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAngle > other.mAngle) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Angle
   *
   * \returns \c true if both Angle are valid and
   *   this Angle is numerically smaller than other.
   * \note the precision of Angle is considered
   */
  bool operator<=(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAngle < other.mAngle) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Angle
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Angle operator+(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    Angle const result(mAngle + other.mAngle);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Angle
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Angle &operator+=(const Angle &other)
  {
    ensureValid();
    other.ensureValid();
    mAngle += other.mAngle;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Angle
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Angle operator-(const Angle &other) const
  {
    ensureValid();
    other.ensureValid();
    Angle const result(mAngle - other.mAngle);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Angle
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Angle operator-=(const Angle &other)
  {
    ensureValid();
    other.ensureValid();
    mAngle -= other.mAngle;
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
  Angle operator*(const double &scalar) const
  {
    ensureValid();
    Angle const result(mAngle * scalar);
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
  Angle operator/(const double &scalar) const
  {
    Angle const scalarAngle(scalar);
    Angle const result(operator/(scalarAngle));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Angle
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Angle is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Angle &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mAngle / other.mAngle;
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
  Angle operator-() const
  {
    ensureValid();
    Angle const result(-mAngle);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_ANGLE_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_ANGLE_EXPLICIT_CONVERSION_ operator double() const
  {
    return mAngle;
  }

  /*!
   * \returns \c true if the Angle in a valid range
   *
   * An Angle value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mAngle);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mAngle) && (mAngle <= cMaxValue);
  }

  /*!
   * \brief ensure that the Angle is valid
   *
   * Throws an std::out_of_range() exception if the Angle
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::Angle)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_ANGLE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Angle value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Angle is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Angle
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Angle(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::Angle)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_ANGLE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Angle value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Angle (i.e. \ref cMinValue)
   */
  static Angle getMin()
  {
    return Angle(cMinValue);
  }

  /*!
   * \brief get maximum valid Angle (i.e. \ref cMaxValue)
   */
  static Angle getMax()
  {
    return Angle(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Angle (i.e. \ref cPrecisionValue)
   */
  static Angle getPrecision()
  {
    return Angle(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mAngle;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Angle as double value
 * \param[in] value Angle value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::Angle operator*(const double &other, ::ad::physics::Angle const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Angle
 */
inline ::ad::physics::Angle fabs(const ::ad::physics::Angle other)
{
  ::ad::physics::Angle const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Angle
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Angle>::lowest()  (\see Angle::getMin())
 * std::numeric_limits<Angle>::max()  (\see Angle::getMax())
 * std::numeric_limits<Angle>::epsilon()  (\see Angle::getPrecision())
 */
template <> class numeric_limits<::ad::physics::Angle> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::Angle lowest()
  {
    return ::ad::physics::Angle::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::Angle max()
  {
    return ::ad::physics::Angle::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::Angle epsilon()
  {
    return ::ad::physics::Angle::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ANGLE
#define GEN_GUARD_AD_PHYSICS_ANGLE
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
 * \param[in] _value Angle value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Angle const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Angle
 */
inline std::string to_string(::ad::physics::Angle const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ANGLE
