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
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace point
 *
 * Handling geographic positions in different coordinate systems
 */
namespace point {

/*!
 * \brief Define to indicate whether throwing exceptions is enabled
 */
#define AD_MAP_POINT_ENUHEADING_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_MAP_POINT_ENUHEADING_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_MAP_POINT_ENUHEADING_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType ENUHeading
 *
 * Angle in ENU coordinate system as angle measured from East to North axis (yaw) in radians.
 * The unit is: Radian
 */
class ENUHeading
{
public:
  /*!
   * \brief constant defining the minimum valid ENUHeading value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid ENUHeading value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed ENUHeading value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of ENUHeading is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  ENUHeading()
    : mENUHeading(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_POINT_ENUHEADING_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_MAP_POINT_ENUHEADING_EXPLICIT_CONVERSION_ ENUHeading(double const iENUHeading)
    : mENUHeading(iENUHeading)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  ENUHeading(const ENUHeading &other) = default;

  /*!
   * \brief standard move constructor
   */
  ENUHeading(ENUHeading &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Reference to this ENUHeading.
   */
  ENUHeading &operator=(const ENUHeading &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Reference to this ENUHeading.
   */
  ENUHeading &operator=(ENUHeading &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns \c true if both ENUHeading are valid and can be taken as numerically equal
   */
  bool operator==(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mENUHeading - other.mENUHeading) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUHeading.
   *
   * \returns \c true if one of the ENUHeading is not valid or they can be taken as numerically different
   */
  bool operator!=(const ENUHeading &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUHeading.
   *
   * \returns \c true if both ENUHeading are valid and
   *   this ENUHeading is strictly numerically greater than other.
   * \note the precision of ENUHeading is considered
   */
  bool operator>(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mENUHeading > other.mENUHeading) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUHeading.
   *
   * \returns \c true if both ENUHeading are valid and
   *   this ENUHeading is strictly numerically smaller than other.
   * \note the precision of ENUHeading is considered
   */
  bool operator<(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mENUHeading < other.mENUHeading) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUHeading.
   *
   * \returns \c true if both ENUHeading are valid and
   *   this ENUHeading is numerically greater than other.
   * \note the precision of ENUHeading is considered
   */
  bool operator>=(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mENUHeading > other.mENUHeading) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns \c true if both ENUHeading are valid and
   *   this ENUHeading is numerically smaller than other.
   * \note the precision of ENUHeading is considered
   */
  bool operator<=(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mENUHeading < other.mENUHeading) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ENUHeading operator+(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    ENUHeading const result(mENUHeading + other.mENUHeading);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ENUHeading &operator+=(const ENUHeading &other)
  {
    ensureValid();
    other.ensureValid();
    mENUHeading += other.mENUHeading;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ENUHeading operator-(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValid();
    ENUHeading const result(mENUHeading - other.mENUHeading);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ENUHeading operator-=(const ENUHeading &other)
  {
    ensureValid();
    other.ensureValid();
    mENUHeading -= other.mENUHeading;
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
  ENUHeading operator*(const double &scalar) const
  {
    ensureValid();
    ENUHeading const result(mENUHeading * scalar);
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
  ENUHeading operator/(const double &scalar) const
  {
    ENUHeading const scalarENUHeading(scalar);
    ENUHeading const result(operator/(scalarENUHeading));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUHeading
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since ENUHeading is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const ENUHeading &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mENUHeading / other.mENUHeading;
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
  ENUHeading operator-() const
  {
    ensureValid();
    ENUHeading const result(-mENUHeading);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_POINT_ENUHEADING_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_MAP_POINT_ENUHEADING_EXPLICIT_CONVERSION_ operator double() const
  {
    return mENUHeading;
  }

  /*!
   * \returns \c true if the ENUHeading in a valid range
   *
   * An ENUHeading value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mENUHeading);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mENUHeading)
      && (mENUHeading <= cMaxValue);
  }

  /*!
   * \brief ensure that the ENUHeading is valid
   *
   * Throws an std::out_of_range() exception if the ENUHeading
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::map::point::ENUHeading)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ENUHEADING_THROWS_EXCEPTION == 1)
      throw std::out_of_range("ENUHeading value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the ENUHeading is valid and non zero
   *
   * Throws an std::out_of_range() exception if the ENUHeading
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(ENUHeading(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::map::point::ENUHeading)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ENUHEADING_THROWS_EXCEPTION == 1)
      throw std::out_of_range("ENUHeading value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid ENUHeading (i.e. \ref cMinValue)
   */
  static ENUHeading getMin()
  {
    return ENUHeading(cMinValue);
  }

  /*!
   * \brief get maximum valid ENUHeading (i.e. \ref cMaxValue)
   */
  static ENUHeading getMax()
  {
    return ENUHeading(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of ENUHeading (i.e. \ref cPrecisionValue)
   */
  static ENUHeading getPrecision()
  {
    return ENUHeading(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mENUHeading;
};

} // namespace point
} // namespace map
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other ENUHeading as double value
 * \param[in] value ENUHeading value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::map::point::ENUHeading operator*(const double &other, ::ad::map::point::ENUHeading const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for ENUHeading
 */
inline ::ad::map::point::ENUHeading fabs(const ::ad::map::point::ENUHeading other)
{
  ::ad::map::point::ENUHeading const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for ENUHeading
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<ENUHeading>::lowest()  (\see ENUHeading::getMin())
 * std::numeric_limits<ENUHeading>::max()  (\see ENUHeading::getMax())
 * std::numeric_limits<ENUHeading>::epsilon()  (\see ENUHeading::getPrecision())
 */
template <> class numeric_limits<::ad::map::point::ENUHeading> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::point::ENUHeading lowest()
  {
    return ::ad::map::point::ENUHeading::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::point::ENUHeading max()
  {
    return ::ad::map::point::ENUHeading::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::point::ENUHeading epsilon()
  {
    return ::ad::map::point::ENUHeading::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ENUHEADING
#define GEN_GUARD_AD_MAP_POINT_ENUHEADING
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace point
 *
 * Handling geographic positions in different coordinate systems
 */
namespace point {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value ENUHeading value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ENUHeading const &_value)
{
  return os << double(_value);
}

} // namespace point
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ENUHeading
 */
inline std::string to_string(::ad::map::point::ENUHeading const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_POINT_ENUHEADING
