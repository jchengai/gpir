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
#define AD_MAP_POINT_ECEFCOORDINATE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_MAP_POINT_ECEFCOORDINATE_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_MAP_POINT_ECEFCOORDINATE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType ECEFCoordinate
 *
 * equatorial WGS-84 earth radius [m]= 6378137
 * Height of the Mount Everest [m]= 8848;
 * Maximum ECEF Coordinate:
 * WGS84_R + MOUNT_EVEREST ~ 6400000
 * The unit is: Meter
 */
class ECEFCoordinate
{
public:
  /*!
   * \brief constant defining the minimum valid ECEFCoordinate value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid ECEFCoordinate value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed ECEFCoordinate value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of ECEFCoordinate is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  ECEFCoordinate()
    : mECEFCoordinate(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_POINT_ECEFCOORDINATE_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_MAP_POINT_ECEFCOORDINATE_EXPLICIT_CONVERSION_ ECEFCoordinate(double const iECEFCoordinate)
    : mECEFCoordinate(iECEFCoordinate)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  ECEFCoordinate(const ECEFCoordinate &other) = default;

  /*!
   * \brief standard move constructor
   */
  ECEFCoordinate(ECEFCoordinate &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Reference to this ECEFCoordinate.
   */
  ECEFCoordinate &operator=(const ECEFCoordinate &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Reference to this ECEFCoordinate.
   */
  ECEFCoordinate &operator=(ECEFCoordinate &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns \c true if both ECEFCoordinate are valid and can be taken as numerically equal
   */
  bool operator==(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mECEFCoordinate - other.mECEFCoordinate) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFCoordinate.
   *
   * \returns \c true if one of the ECEFCoordinate is not valid or they can be taken as numerically different
   */
  bool operator!=(const ECEFCoordinate &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFCoordinate.
   *
   * \returns \c true if both ECEFCoordinate are valid and
   *   this ECEFCoordinate is strictly numerically greater than other.
   * \note the precision of ECEFCoordinate is considered
   */
  bool operator>(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mECEFCoordinate > other.mECEFCoordinate) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFCoordinate.
   *
   * \returns \c true if both ECEFCoordinate are valid and
   *   this ECEFCoordinate is strictly numerically smaller than other.
   * \note the precision of ECEFCoordinate is considered
   */
  bool operator<(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mECEFCoordinate < other.mECEFCoordinate) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFCoordinate.
   *
   * \returns \c true if both ECEFCoordinate are valid and
   *   this ECEFCoordinate is numerically greater than other.
   * \note the precision of ECEFCoordinate is considered
   */
  bool operator>=(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mECEFCoordinate > other.mECEFCoordinate) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns \c true if both ECEFCoordinate are valid and
   *   this ECEFCoordinate is numerically smaller than other.
   * \note the precision of ECEFCoordinate is considered
   */
  bool operator<=(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mECEFCoordinate < other.mECEFCoordinate) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ECEFCoordinate operator+(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    ECEFCoordinate const result(mECEFCoordinate + other.mECEFCoordinate);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ECEFCoordinate &operator+=(const ECEFCoordinate &other)
  {
    ensureValid();
    other.ensureValid();
    mECEFCoordinate += other.mECEFCoordinate;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ECEFCoordinate operator-(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValid();
    ECEFCoordinate const result(mECEFCoordinate - other.mECEFCoordinate);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  ECEFCoordinate operator-=(const ECEFCoordinate &other)
  {
    ensureValid();
    other.ensureValid();
    mECEFCoordinate -= other.mECEFCoordinate;
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
  ECEFCoordinate operator*(const double &scalar) const
  {
    ensureValid();
    ECEFCoordinate const result(mECEFCoordinate * scalar);
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
  ECEFCoordinate operator/(const double &scalar) const
  {
    ECEFCoordinate const scalarECEFCoordinate(scalar);
    ECEFCoordinate const result(operator/(scalarECEFCoordinate));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ECEFCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since ECEFCoordinate is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const ECEFCoordinate &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mECEFCoordinate / other.mECEFCoordinate;
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
  ECEFCoordinate operator-() const
  {
    ensureValid();
    ECEFCoordinate const result(-mECEFCoordinate);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_POINT_ECEFCOORDINATE_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_MAP_POINT_ECEFCOORDINATE_EXPLICIT_CONVERSION_ operator double() const
  {
    return mECEFCoordinate;
  }

  /*!
   * \returns \c true if the ECEFCoordinate in a valid range
   *
   * An ECEFCoordinate value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mECEFCoordinate);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mECEFCoordinate)
      && (mECEFCoordinate <= cMaxValue);
  }

  /*!
   * \brief ensure that the ECEFCoordinate is valid
   *
   * Throws an std::out_of_range() exception if the ECEFCoordinate
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::map::point::ECEFCoordinate)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ECEFCOORDINATE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("ECEFCoordinate value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the ECEFCoordinate is valid and non zero
   *
   * Throws an std::out_of_range() exception if the ECEFCoordinate
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(ECEFCoordinate(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::map::point::ECEFCoordinate)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ECEFCOORDINATE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("ECEFCoordinate value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid ECEFCoordinate (i.e. \ref cMinValue)
   */
  static ECEFCoordinate getMin()
  {
    return ECEFCoordinate(cMinValue);
  }

  /*!
   * \brief get maximum valid ECEFCoordinate (i.e. \ref cMaxValue)
   */
  static ECEFCoordinate getMax()
  {
    return ECEFCoordinate(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of ECEFCoordinate (i.e. \ref cPrecisionValue)
   */
  static ECEFCoordinate getPrecision()
  {
    return ECEFCoordinate(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mECEFCoordinate;
};

} // namespace point
} // namespace map
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other ECEFCoordinate as double value
 * \param[in] value ECEFCoordinate value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::map::point::ECEFCoordinate operator*(const double &other, ::ad::map::point::ECEFCoordinate const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for ECEFCoordinate
 */
inline ::ad::map::point::ECEFCoordinate fabs(const ::ad::map::point::ECEFCoordinate other)
{
  ::ad::map::point::ECEFCoordinate const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for ECEFCoordinate
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<ECEFCoordinate>::lowest()  (\see ECEFCoordinate::getMin())
 * std::numeric_limits<ECEFCoordinate>::max()  (\see ECEFCoordinate::getMax())
 * std::numeric_limits<ECEFCoordinate>::epsilon()  (\see ECEFCoordinate::getPrecision())
 */
template <> class numeric_limits<::ad::map::point::ECEFCoordinate> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::point::ECEFCoordinate lowest()
  {
    return ::ad::map::point::ECEFCoordinate::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::point::ECEFCoordinate max()
  {
    return ::ad::map::point::ECEFCoordinate::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::point::ECEFCoordinate epsilon()
  {
    return ::ad::map::point::ECEFCoordinate::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ECEFCOORDINATE
#define GEN_GUARD_AD_MAP_POINT_ECEFCOORDINATE
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
 * \param[in] _value ECEFCoordinate value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ECEFCoordinate const &_value)
{
  return os << double(_value);
}

} // namespace point
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ECEFCoordinate
 */
inline std::string to_string(::ad::map::point::ECEFCoordinate const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_POINT_ECEFCOORDINATE
