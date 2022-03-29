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
#define AD_MAP_POINT_ALTITUDE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_MAP_POINT_ALTITUDE_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_MAP_POINT_ALTITUDE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType Altitude
 *
 * WGS-84 Altitude
 * Depth of the Mariana Trench [m] 10994
 * Height of the Mount Everest [m] 8848
 * The unit is: Meter
 */
class Altitude
{
public:
  /*!
   * \brief constant defining the minimum valid Altitude value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Altitude value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Altitude value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Altitude is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Altitude()
    : mAltitude(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_POINT_ALTITUDE_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_MAP_POINT_ALTITUDE_EXPLICIT_CONVERSION_ Altitude(double const iAltitude)
    : mAltitude(iAltitude)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Altitude(const Altitude &other) = default;

  /*!
   * \brief standard move constructor
   */
  Altitude(Altitude &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Reference to this Altitude.
   */
  Altitude &operator=(const Altitude &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Reference to this Altitude.
   */
  Altitude &operator=(Altitude &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Altitude
   *
   * \returns \c true if both Altitude are valid and can be taken as numerically equal
   */
  bool operator==(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mAltitude - other.mAltitude) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Altitude.
   *
   * \returns \c true if one of the Altitude is not valid or they can be taken as numerically different
   */
  bool operator!=(const Altitude &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Altitude.
   *
   * \returns \c true if both Altitude are valid and
   *   this Altitude is strictly numerically greater than other.
   * \note the precision of Altitude is considered
   */
  bool operator>(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAltitude > other.mAltitude) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Altitude.
   *
   * \returns \c true if both Altitude are valid and
   *   this Altitude is strictly numerically smaller than other.
   * \note the precision of Altitude is considered
   */
  bool operator<(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mAltitude < other.mAltitude) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Altitude.
   *
   * \returns \c true if both Altitude are valid and
   *   this Altitude is numerically greater than other.
   * \note the precision of Altitude is considered
   */
  bool operator>=(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAltitude > other.mAltitude) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Altitude
   *
   * \returns \c true if both Altitude are valid and
   *   this Altitude is numerically smaller than other.
   * \note the precision of Altitude is considered
   */
  bool operator<=(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mAltitude < other.mAltitude) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Altitude operator+(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    Altitude const result(mAltitude + other.mAltitude);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Altitude &operator+=(const Altitude &other)
  {
    ensureValid();
    other.ensureValid();
    mAltitude += other.mAltitude;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Altitude operator-(const Altitude &other) const
  {
    ensureValid();
    other.ensureValid();
    Altitude const result(mAltitude - other.mAltitude);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Altitude operator-=(const Altitude &other)
  {
    ensureValid();
    other.ensureValid();
    mAltitude -= other.mAltitude;
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
  Altitude operator*(const double &scalar) const
  {
    ensureValid();
    Altitude const result(mAltitude * scalar);
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
  Altitude operator/(const double &scalar) const
  {
    Altitude const scalarAltitude(scalar);
    Altitude const result(operator/(scalarAltitude));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Altitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Altitude is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Altitude &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mAltitude / other.mAltitude;
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
  Altitude operator-() const
  {
    ensureValid();
    Altitude const result(-mAltitude);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_POINT_ALTITUDE_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_MAP_POINT_ALTITUDE_EXPLICIT_CONVERSION_ operator double() const
  {
    return mAltitude;
  }

  /*!
   * \returns \c true if the Altitude in a valid range
   *
   * An Altitude value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mAltitude);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mAltitude)
      && (mAltitude <= cMaxValue);
  }

  /*!
   * \brief ensure that the Altitude is valid
   *
   * Throws an std::out_of_range() exception if the Altitude
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::map::point::Altitude)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ALTITUDE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Altitude value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Altitude is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Altitude
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Altitude(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::map::point::Altitude)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ALTITUDE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Altitude value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Altitude (i.e. \ref cMinValue)
   */
  static Altitude getMin()
  {
    return Altitude(cMinValue);
  }

  /*!
   * \brief get maximum valid Altitude (i.e. \ref cMaxValue)
   */
  static Altitude getMax()
  {
    return Altitude(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Altitude (i.e. \ref cPrecisionValue)
   */
  static Altitude getPrecision()
  {
    return Altitude(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mAltitude;
};

} // namespace point
} // namespace map
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Altitude as double value
 * \param[in] value Altitude value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::map::point::Altitude operator*(const double &other, ::ad::map::point::Altitude const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Altitude
 */
inline ::ad::map::point::Altitude fabs(const ::ad::map::point::Altitude other)
{
  ::ad::map::point::Altitude const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Altitude
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Altitude>::lowest()  (\see Altitude::getMin())
 * std::numeric_limits<Altitude>::max()  (\see Altitude::getMax())
 * std::numeric_limits<Altitude>::epsilon()  (\see Altitude::getPrecision())
 */
template <> class numeric_limits<::ad::map::point::Altitude> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::point::Altitude lowest()
  {
    return ::ad::map::point::Altitude::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::point::Altitude max()
  {
    return ::ad::map::point::Altitude::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::point::Altitude epsilon()
  {
    return ::ad::map::point::Altitude::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ALTITUDE
#define GEN_GUARD_AD_MAP_POINT_ALTITUDE
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
 * \param[in] _value Altitude value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Altitude const &_value)
{
  return os << double(_value);
}

} // namespace point
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Altitude
 */
inline std::string to_string(::ad::map::point::Altitude const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_POINT_ALTITUDE
