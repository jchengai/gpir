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
#define AD_MAP_POINT_ENUCOORDINATE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "only explicit
 * conversion".
 */
#define _AD_MAP_POINT_ENUCOORDINATE_EXPLICIT_CONVERSION_ explicit
#else
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "implicit
 * conversion allowed".
 */
#define _AD_MAP_POINT_ENUCOORDINATE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType ENUCoordinate
 *
 * Maximum ENU coordinate (16 km): 16384
 * The unit is: Meter
 */
class ENUCoordinate {
 public:
  /*!
   * \brief constant defining the minimum valid ENUCoordinate value (used in
   * isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid ENUCoordinate value (used in
   * isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed ENUCoordinate value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of ENUCoordinate is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  ENUCoordinate() : mENUCoordinate(std::numeric_limits<double>::quiet_NaN()) {}

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_POINT_ENUCOORDINATE_EXPLICIT_CONVERSION\_ defines, if
   * only an explicit conversion is allowed.
   */
  _AD_MAP_POINT_ENUCOORDINATE_EXPLICIT_CONVERSION_ ENUCoordinate(
      double const iENUCoordinate)
      : mENUCoordinate(iENUCoordinate) {}

  /*!
   * \brief standard copy constructor
   */
  ENUCoordinate(const ENUCoordinate &other) = default;

  /*!
   * \brief standard move constructor
   */
  ENUCoordinate(ENUCoordinate &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Reference to this ENUCoordinate.
   */
  ENUCoordinate &operator=(const ENUCoordinate &other) = default;

  void operator=(const double coordinate) { mENUCoordinate = coordinate; }

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Reference to this ENUCoordinate.
   */
  ENUCoordinate &operator=(ENUCoordinate &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns \c true if both ENUCoordinate are valid and can be taken as
   * numerically equal
   */
  bool operator==(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    return std::fabs(mENUCoordinate - other.mENUCoordinate) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUCoordinate.
   *
   * \returns \c true if one of the ENUCoordinate is not valid or they can be
   * taken as numerically different
   */
  bool operator!=(const ENUCoordinate &other) const {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUCoordinate.
   *
   * \returns \c true if both ENUCoordinate are valid and
   *   this ENUCoordinate is strictly numerically greater than other.
   * \note the precision of ENUCoordinate is considered
   */
  bool operator>(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    return (mENUCoordinate > other.mENUCoordinate) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUCoordinate.
   *
   * \returns \c true if both ENUCoordinate are valid and
   *   this ENUCoordinate is strictly numerically smaller than other.
   * \note the precision of ENUCoordinate is considered
   */
  bool operator<(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    return (mENUCoordinate < other.mENUCoordinate) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUCoordinate.
   *
   * \returns \c true if both ENUCoordinate are valid and
   *   this ENUCoordinate is numerically greater than other.
   * \note the precision of ENUCoordinate is considered
   */
  bool operator>=(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    return ((mENUCoordinate > other.mENUCoordinate) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns \c true if both ENUCoordinate are valid and
   *   this ENUCoordinate is numerically smaller than other.
   * \note the precision of ENUCoordinate is considered
   */
  bool operator<=(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    return ((mENUCoordinate < other.mENUCoordinate) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ENUCoordinate operator+(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    ENUCoordinate const result(mENUCoordinate + other.mENUCoordinate);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ENUCoordinate &operator+=(const ENUCoordinate &other) {
    ensureValid();
    other.ensureValid();
    mENUCoordinate += other.mENUCoordinate;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ENUCoordinate operator-(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValid();
    ENUCoordinate const result(mENUCoordinate - other.mENUCoordinate);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  ENUCoordinate operator-=(const ENUCoordinate &other) {
    ensureValid();
    other.ensureValid();
    mENUCoordinate -= other.mENUCoordinate;
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
  ENUCoordinate operator*(const double &scalar) const {
    ensureValid();
    ENUCoordinate const result(mENUCoordinate * scalar);
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
  ENUCoordinate operator/(const double &scalar) const {
    ENUCoordinate const scalarENUCoordinate(scalar);
    ENUCoordinate const result(operator/(scalarENUCoordinate));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other ENUCoordinate
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid or other is zero \note since
   * ENUCoordinate is a type with physical unit, the division results in the
   * dimensionless type.
   */
  double operator/(const ENUCoordinate &other) const {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mENUCoordinate / other.mENUCoordinate;
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
  ENUCoordinate operator-() const {
    ensureValid();
    ENUCoordinate const result(-mENUCoordinate);
    result.ensureValid();  // LCOV_EXCL_BR_LINE Some types do not throw an
                           // exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_POINT_ENUCOORDINATE_EXPLICIT_CONVERSION\_ defines, if
   * only explicit calls are allowed.
   */
  _AD_MAP_POINT_ENUCOORDINATE_EXPLICIT_CONVERSION_ operator double() const {
    return mENUCoordinate;
  }

  /*!
   * \returns \c true if the ENUCoordinate in a valid range
   *
   * An ENUCoordinate value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const {
    auto const valueClass = std::fpclassify(mENUCoordinate);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) &&
           (cMinValue <= mENUCoordinate) && (mENUCoordinate <= cMaxValue);
  }

  /*!
   * \brief ensure that the ENUCoordinate is valid
   *
   * Throws an std::out_of_range() exception if the ENUCoordinate
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const {
    if (!isValid()) {
      spdlog::info(
          "ensureValid(::ad::map::point::ENUCoordinate)>> {} value out of "
          "range",
          *this);  // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ENUCOORDINATE_THROWS_EXCEPTION == 1)
      throw std::out_of_range(
          "ENUCoordinate value out of range");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the ENUCoordinate is valid and non zero
   *
   * Throws an std::out_of_range() exception if the ENUCoordinate
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const {
    ensureValid();
    if (operator==(ENUCoordinate(0.)))  // LCOV_EXCL_BR_LINE
    {
      spdlog::info(
          "ensureValid(::ad::map::point::ENUCoordinate)>> {} value is zero",
          *this);  // LCOV_EXCL_BR_LINE
#if (AD_MAP_POINT_ENUCOORDINATE_THROWS_EXCEPTION == 1)
      throw std::out_of_range(
          "ENUCoordinate value is zero");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid ENUCoordinate (i.e. \ref cMinValue)
   */
  static ENUCoordinate getMin() { return ENUCoordinate(cMinValue); }

  /*!
   * \brief get maximum valid ENUCoordinate (i.e. \ref cMaxValue)
   */
  static ENUCoordinate getMax() { return ENUCoordinate(cMaxValue); }

  /*!
   * \brief get assumed accuracy of ENUCoordinate (i.e. \ref cPrecisionValue)
   */
  static ENUCoordinate getPrecision() { return ENUCoordinate(cPrecisionValue); }

  /*!
   * \brief get double value
   */
  inline double value() const { return mENUCoordinate; }

 private:
  /*!
   * \brief the actual value of the type
   */
  double mENUCoordinate;
};

}  // namespace point
}  // namespace map
}  // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other ENUCoordinate as double value
 * \param[in] value ENUCoordinate value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::map::point::ENUCoordinate operator*(
    const double &other, ::ad::map::point::ENUCoordinate const &value) {
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for ENUCoordinate
 */
inline ::ad::map::point::ENUCoordinate fabs(
    const ::ad::map::point::ENUCoordinate other) {
  ::ad::map::point::ENUCoordinate const result(
      std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for ENUCoordinate
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<ENUCoordinate>::lowest()  (\see ENUCoordinate::getMin())
 * std::numeric_limits<ENUCoordinate>::max()  (\see ENUCoordinate::getMax())
 * std::numeric_limits<ENUCoordinate>::epsilon()  (\see
 * ENUCoordinate::getPrecision())
 */
template <>
class numeric_limits<::ad::map::point::ENUCoordinate>
    : public numeric_limits<double> {
 public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::point::ENUCoordinate lowest() {
    return ::ad::map::point::ENUCoordinate::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::point::ENUCoordinate max() {
    return ::ad::map::point::ENUCoordinate::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::point::ENUCoordinate epsilon() {
    return ::ad::map::point::ENUCoordinate::getPrecision();
  }
};

}  // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ENUCOORDINATE
#define GEN_GUARD_AD_MAP_POINT_ENUCOORDINATE
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
 * \param[in] _value ENUCoordinate value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ENUCoordinate const &_value) {
  return os << double(_value);
}

}  // namespace point
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ENUCoordinate
 */
inline std::string to_string(::ad::map::point::ENUCoordinate const &value) {
  return to_string(static_cast<double>(value));
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_ENUCOORDINATE
