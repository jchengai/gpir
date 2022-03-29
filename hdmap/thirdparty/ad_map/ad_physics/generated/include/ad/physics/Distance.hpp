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
#define AD_PHYSICS_DISTANCE_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "only explicit
 * conversion".
 */
#define _AD_PHYSICS_DISTANCE_EXPLICIT_CONVERSION_ explicit
#else
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "implicit
 * conversion allowed".
 */
#define _AD_PHYSICS_DISTANCE_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief Forward declaration of DistanceSquared
 *
 * Since Distance is defined explicitly as a physical type we have to consider
 * this within operations. Therefore this squared type is defined.
 */
class DistanceSquared;

/*!
 * \brief DataType Distance
 *
 * The length of a specific path traveled between two points.
 * The unit is: Meter
 */
class Distance {
 public:
  /*!
   * \brief constant defining the minimum valid Distance value (used in
   * isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Distance value (used in
   * isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Distance value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Distance is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Distance() : mDistance(std::numeric_limits<double>::quiet_NaN()) {}

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_DISTANCE_EXPLICIT_CONVERSION\_ defines, if only an
   * explicit conversion is allowed.
   */
  _AD_PHYSICS_DISTANCE_EXPLICIT_CONVERSION_ Distance(double const iDistance)
      : mDistance(iDistance) {}

  /*!
   * \brief standard copy constructor
   */
  Distance(const Distance &other) = default;

  /*!
   * \brief standard move constructor
   */
  Distance(Distance &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Distance
   *
   * \returns Reference to this Distance.
   */
  Distance &operator=(const Distance &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Distance
   *
   * \returns Reference to this Distance.
   */
  Distance &operator=(Distance &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance
   *
   * \returns \c true if both Distance are valid and can be taken as numerically
   * equal
   */
  bool operator==(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    return std::fabs(mDistance - other.mDistance) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance.
   *
   * \returns \c true if one of the Distance is not valid or they can be taken
   * as numerically different
   */
  bool operator!=(const Distance &other) const { return !operator==(other); }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance.
   *
   * \returns \c true if both Distance are valid and
   *   this Distance is strictly numerically greater than other.
   * \note the precision of Distance is considered
   */
  bool operator>(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    return (mDistance > other.mDistance) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance.
   *
   * \returns \c true if both Distance are valid and
   *   this Distance is strictly numerically smaller than other.
   * \note the precision of Distance is considered
   */
  bool operator<(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    return (mDistance < other.mDistance) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance.
   *
   * \returns \c true if both Distance are valid and
   *   this Distance is numerically greater than other.
   * \note the precision of Distance is considered
   */
  bool operator>=(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    return ((mDistance > other.mDistance) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance
   *
   * \returns \c true if both Distance are valid and
   *   this Distance is numerically smaller than other.
   * \note the precision of Distance is considered
   */
  bool operator<=(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    return ((mDistance < other.mDistance) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Distance
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  Distance operator+(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    Distance const result(mDistance + other.mDistance);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Distance
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  Distance &operator+=(const Distance &other) {
    ensureValid();
    other.ensureValid();
    mDistance += other.mDistance;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Distance
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  Distance operator-(const Distance &other) const {
    ensureValid();
    other.ensureValid();
    Distance const result(mDistance - other.mDistance);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Distance
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  Distance operator-=(const Distance &other) {
    ensureValid();
    other.ensureValid();
    mDistance -= other.mDistance;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Distance
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid \note since Distance is a type
   * with physical unit, the multiplication results in the Squared type.
   */
  DistanceSquared operator*(const Distance &other) const;

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
  Distance operator*(const double &scalar) const {
    ensureValid();
    Distance const result(mDistance * scalar);
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
  Distance operator/(const double &scalar) const {
    Distance const scalarDistance(scalar);
    Distance const result(operator/(scalarDistance));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Distance
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid or other is zero \note since
   * Distance is a type with physical unit, the division results in the
   * dimensionless type.
   */
  double operator/(const Distance &other) const {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mDistance / other.mDistance;
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
  Distance operator-() const {
    ensureValid();
    Distance const result(-mDistance);
    result.ensureValid();  // LCOV_EXCL_BR_LINE Some types do not throw an
                           // exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_DISTANCE_EXPLICIT_CONVERSION\_ defines, if only
   * explicit calls are allowed.
   */
  _AD_PHYSICS_DISTANCE_EXPLICIT_CONVERSION_ operator double() const {
    return mDistance;
  }

  /*!
   * \returns \c true if the Distance in a valid range
   *
   * An Distance value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const {
    auto const valueClass = std::fpclassify(mDistance);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) &&
           (cMinValue <= mDistance) && (mDistance <= cMaxValue);
  }

  /*!
   * \brief ensure that the Distance is valid
   *
   * Throws an std::out_of_range() exception if the Distance
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const {
    if (!isValid()) {
      spdlog::info(
          "ensureValid(::ad::physics::Distance)>> {} value out of range",
          *this);  // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_DISTANCE_THROWS_EXCEPTION == 1)
      throw std::out_of_range(
          "Distance value out of range");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Distance is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Distance
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const {
    ensureValid();
    if (operator==(Distance(0.)))  // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::Distance)>> {} value is zero",
                   *this);  // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_DISTANCE_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Distance value is zero");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Distance (i.e. \ref cMinValue)
   */
  static Distance getMin() { return Distance(cMinValue); }

  /*!
   * \brief get maximum valid Distance (i.e. \ref cMaxValue)
   */
  static Distance getMax() { return Distance(cMaxValue); }

  /*!
   * \brief get assumed accuracy of Distance (i.e. \ref cPrecisionValue)
   */
  static Distance getPrecision() { return Distance(cPrecisionValue); }

 private:
  /*!
   * \brief the actual value of the type
   */
  double mDistance;
};

}  // namespace physics
}  // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Distance as double value
 * \param[in] value Distance value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::Distance operator*(const double &other,
                                         ::ad::physics::Distance const &value) {
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Distance
 */
inline ::ad::physics::Distance fabs(const ::ad::physics::Distance other) {
  ::ad::physics::Distance const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Distance
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Distance>::lowest()  (\see Distance::getMin())
 * std::numeric_limits<Distance>::max()  (\see Distance::getMax())
 * std::numeric_limits<Distance>::epsilon()  (\see Distance::getPrecision())
 */
template <>
class numeric_limits<::ad::physics::Distance> : public numeric_limits<double> {
 public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::Distance lowest() {
    return ::ad::physics::Distance::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::Distance max() {
    return ::ad::physics::Distance::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::Distance epsilon() {
    return ::ad::physics::Distance::getPrecision();
  }
};

}  // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DISTANCE
#define GEN_GUARD_AD_PHYSICS_DISTANCE
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
 * \param[in] _value Distance value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Distance const &_value) {
  return os << double(_value);
}

}  // namespace physics
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Distance
 */
inline std::string to_string(::ad::physics::Distance const &value) {
  return to_string(static_cast<double>(value));
}
}  // namespace std
#endif  // GEN_GUARD_AD_PHYSICS_DISTANCE
