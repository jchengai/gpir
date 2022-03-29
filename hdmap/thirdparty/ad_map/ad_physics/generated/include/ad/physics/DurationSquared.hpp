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
#define AD_PHYSICS_DURATIONSQUARED_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_DURATIONSQUARED_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_DURATIONSQUARED_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief Forward declaration of Duration
 *
 * Since DurationSquared is defined explicitly as a physical type we have to consider this
 * within operations. Therefore this square-rooted type is defined.
 */
class Duration;

/*!
 * \brief DataType DurationSquared
 *
 * DurationSquared represents a squared Duration
 * The unit is: SecondSquared
 */
class DurationSquared
{
public:
  /*!
   * \brief constant defining the minimum valid DurationSquared value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid DurationSquared value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed DurationSquared value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of DurationSquared is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  DurationSquared()
    : mDurationSquared(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_DURATIONSQUARED_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_DURATIONSQUARED_EXPLICIT_CONVERSION_ DurationSquared(double const iDurationSquared)
    : mDurationSquared(iDurationSquared)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  DurationSquared(const DurationSquared &other) = default;

  /*!
   * \brief standard move constructor
   */
  DurationSquared(DurationSquared &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Reference to this DurationSquared.
   */
  DurationSquared &operator=(const DurationSquared &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Reference to this DurationSquared.
   */
  DurationSquared &operator=(DurationSquared &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns \c true if both DurationSquared are valid and can be taken as numerically equal
   */
  bool operator==(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mDurationSquared - other.mDurationSquared) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other DurationSquared.
   *
   * \returns \c true if one of the DurationSquared is not valid or they can be taken as numerically different
   */
  bool operator!=(const DurationSquared &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other DurationSquared.
   *
   * \returns \c true if both DurationSquared are valid and
   *   this DurationSquared is strictly numerically greater than other.
   * \note the precision of DurationSquared is considered
   */
  bool operator>(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mDurationSquared > other.mDurationSquared) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other DurationSquared.
   *
   * \returns \c true if both DurationSquared are valid and
   *   this DurationSquared is strictly numerically smaller than other.
   * \note the precision of DurationSquared is considered
   */
  bool operator<(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mDurationSquared < other.mDurationSquared) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other DurationSquared.
   *
   * \returns \c true if both DurationSquared are valid and
   *   this DurationSquared is numerically greater than other.
   * \note the precision of DurationSquared is considered
   */
  bool operator>=(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mDurationSquared > other.mDurationSquared) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns \c true if both DurationSquared are valid and
   *   this DurationSquared is numerically smaller than other.
   * \note the precision of DurationSquared is considered
   */
  bool operator<=(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mDurationSquared < other.mDurationSquared) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  DurationSquared operator+(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    DurationSquared const result(mDurationSquared + other.mDurationSquared);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  DurationSquared &operator+=(const DurationSquared &other)
  {
    ensureValid();
    other.ensureValid();
    mDurationSquared += other.mDurationSquared;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  DurationSquared operator-(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValid();
    DurationSquared const result(mDurationSquared - other.mDurationSquared);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  DurationSquared operator-=(const DurationSquared &other)
  {
    ensureValid();
    other.ensureValid();
    mDurationSquared -= other.mDurationSquared;
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
  DurationSquared operator*(const double &scalar) const
  {
    ensureValid();
    DurationSquared const result(mDurationSquared * scalar);
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
  DurationSquared operator/(const double &scalar) const
  {
    DurationSquared const scalarDurationSquared(scalar);
    DurationSquared const result(operator/(scalarDurationSquared));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other DurationSquared
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since DurationSquared is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const DurationSquared &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mDurationSquared / other.mDurationSquared;
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
  DurationSquared operator-() const
  {
    ensureValid();
    DurationSquared const result(-mDurationSquared);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_DURATIONSQUARED_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_DURATIONSQUARED_EXPLICIT_CONVERSION_ operator double() const
  {
    return mDurationSquared;
  }

  /*!
   * \returns \c true if the DurationSquared in a valid range
   *
   * An DurationSquared value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mDurationSquared);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mDurationSquared)
      && (mDurationSquared <= cMaxValue);
  }

  /*!
   * \brief ensure that the DurationSquared is valid
   *
   * Throws an std::out_of_range() exception if the DurationSquared
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::DurationSquared)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_DURATIONSQUARED_THROWS_EXCEPTION == 1)
      throw std::out_of_range("DurationSquared value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the DurationSquared is valid and non zero
   *
   * Throws an std::out_of_range() exception if the DurationSquared
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(DurationSquared(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::DurationSquared)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_DURATIONSQUARED_THROWS_EXCEPTION == 1)
      throw std::out_of_range("DurationSquared value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid DurationSquared (i.e. \ref cMinValue)
   */
  static DurationSquared getMin()
  {
    return DurationSquared(cMinValue);
  }

  /*!
   * \brief get maximum valid DurationSquared (i.e. \ref cMaxValue)
   */
  static DurationSquared getMax()
  {
    return DurationSquared(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of DurationSquared (i.e. \ref cPrecisionValue)
   */
  static DurationSquared getPrecision()
  {
    return DurationSquared(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mDurationSquared;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other DurationSquared as double value
 * \param[in] value DurationSquared value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::DurationSquared operator*(const double &other, ::ad::physics::DurationSquared const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for DurationSquared
 */
inline ::ad::physics::DurationSquared fabs(const ::ad::physics::DurationSquared other)
{
  ::ad::physics::DurationSquared const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for DurationSquared
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<DurationSquared>::lowest()  (\see DurationSquared::getMin())
 * std::numeric_limits<DurationSquared>::max()  (\see DurationSquared::getMax())
 * std::numeric_limits<DurationSquared>::epsilon()  (\see DurationSquared::getPrecision())
 */
template <> class numeric_limits<::ad::physics::DurationSquared> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::DurationSquared lowest()
  {
    return ::ad::physics::DurationSquared::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::DurationSquared max()
  {
    return ::ad::physics::DurationSquared::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::DurationSquared epsilon()
  {
    return ::ad::physics::DurationSquared::getPrecision();
  }
};

/*!
 * \brief overload of the std::sqrt for DurationSquared
 *
 * The square root of a squared type is basic type: Duration.
 */
::ad::physics::Duration sqrt(::ad::physics::DurationSquared const other);

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DURATIONSQUARED
#define GEN_GUARD_AD_PHYSICS_DURATIONSQUARED
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
 * \param[in] _value DurationSquared value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, DurationSquared const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for DurationSquared
 */
inline std::string to_string(::ad::physics::DurationSquared const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_DURATIONSQUARED
