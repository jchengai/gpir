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
#define AD_MAP_POINT_LATITUDE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_MAP_POINT_LATITUDE_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_MAP_POINT_LATITUDE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType Latitude
 *
 * WGS-84 Latitude
 * The unit is: Degree
 */
class Latitude
{
public:
  /*!
   * \brief constant defining the minimum valid Latitude value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Latitude value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Latitude value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Latitude is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Latitude()
    : mLatitude(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_POINT_LATITUDE_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_MAP_POINT_LATITUDE_EXPLICIT_CONVERSION_ Latitude(double const iLatitude)
    : mLatitude(iLatitude)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Latitude(const Latitude &other) = default;

  /*!
   * \brief standard move constructor
   */
  Latitude(Latitude &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Reference to this Latitude.
   */
  Latitude &operator=(const Latitude &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Reference to this Latitude.
   */
  Latitude &operator=(Latitude &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Latitude
   *
   * \returns \c true if both Latitude are valid and can be taken as numerically equal
   */
  bool operator==(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mLatitude - other.mLatitude) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Latitude.
   *
   * \returns \c true if one of the Latitude is not valid or they can be taken as numerically different
   */
  bool operator!=(const Latitude &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Latitude.
   *
   * \returns \c true if both Latitude are valid and
   *   this Latitude is strictly numerically greater than other.
   * \note the precision of Latitude is considered
   */
  bool operator>(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mLatitude > other.mLatitude) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Latitude.
   *
   * \returns \c true if both Latitude are valid and
   *   this Latitude is strictly numerically smaller than other.
   * \note the precision of Latitude is considered
   */
  bool operator<(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mLatitude < other.mLatitude) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Latitude.
   *
   * \returns \c true if both Latitude are valid and
   *   this Latitude is numerically greater than other.
   * \note the precision of Latitude is considered
   */
  bool operator>=(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mLatitude > other.mLatitude) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Latitude
   *
   * \returns \c true if both Latitude are valid and
   *   this Latitude is numerically smaller than other.
   * \note the precision of Latitude is considered
   */
  bool operator<=(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mLatitude < other.mLatitude) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Latitude operator+(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    Latitude const result(mLatitude + other.mLatitude);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Latitude &operator+=(const Latitude &other)
  {
    ensureValid();
    other.ensureValid();
    mLatitude += other.mLatitude;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Latitude operator-(const Latitude &other) const
  {
    ensureValid();
    other.ensureValid();
    Latitude const result(mLatitude - other.mLatitude);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Latitude operator-=(const Latitude &other)
  {
    ensureValid();
    other.ensureValid();
    mLatitude -= other.mLatitude;
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
  Latitude operator*(const double &scalar) const
  {
    ensureValid();
    Latitude const result(mLatitude * scalar);
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
  Latitude operator/(const double &scalar) const
  {
    Latitude const scalarLatitude(scalar);
    Latitude const result(operator/(scalarLatitude));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Latitude
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Latitude is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Latitude &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mLatitude / other.mLatitude;
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
  Latitude operator-() const
  {
    ensureValid();
    Latitude const result(-mLatitude);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_POINT_LATITUDE_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_MAP_POINT_LATITUDE_EXPLICIT_CONVERSION_ operator double() const
  {
    return mLatitude;
  }

  /*!
   * \returns \c true if the Latitude in a valid range
   *
   * An Latitude value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mLatitude);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mLatitude)
      && (mLatitude <= cMaxValue);
  }

  /*!
   * \brief ensure that the Latitude is valid
   *
   * Throws an std::out_of_range() exception if the Latitude
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::map::point::Latitude)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_LATITUDE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Latitude value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Latitude is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Latitude
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Latitude(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::map::point::Latitude)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_LATITUDE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Latitude value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Latitude (i.e. \ref cMinValue)
   */
  static Latitude getMin()
  {
    return Latitude(cMinValue);
  }

  /*!
   * \brief get maximum valid Latitude (i.e. \ref cMaxValue)
   */
  static Latitude getMax()
  {
    return Latitude(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Latitude (i.e. \ref cPrecisionValue)
   */
  static Latitude getPrecision()
  {
    return Latitude(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mLatitude;
};

} // namespace point
} // namespace map
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Latitude as double value
 * \param[in] value Latitude value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::map::point::Latitude operator*(const double &other, ::ad::map::point::Latitude const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Latitude
 */
inline ::ad::map::point::Latitude fabs(const ::ad::map::point::Latitude other)
{
  ::ad::map::point::Latitude const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Latitude
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Latitude>::lowest()  (\see Latitude::getMin())
 * std::numeric_limits<Latitude>::max()  (\see Latitude::getMax())
 * std::numeric_limits<Latitude>::epsilon()  (\see Latitude::getPrecision())
 */
template <> class numeric_limits<::ad::map::point::Latitude> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::point::Latitude lowest()
  {
    return ::ad::map::point::Latitude::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::point::Latitude max()
  {
    return ::ad::map::point::Latitude::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::point::Latitude epsilon()
  {
    return ::ad::map::point::Latitude::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_LATITUDE
#define GEN_GUARD_AD_MAP_POINT_LATITUDE
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
 * \param[in] _value Latitude value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Latitude const &_value)
{
  return os << double(_value);
}

} // namespace point
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Latitude
 */
inline std::string to_string(::ad::map::point::Latitude const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_POINT_LATITUDE
