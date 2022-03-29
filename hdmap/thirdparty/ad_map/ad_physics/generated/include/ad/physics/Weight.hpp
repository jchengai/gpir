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
#define AD_PHYSICS_WEIGHT_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_WEIGHT_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_WEIGHT_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType Weight
 *
 * Weight represents the mass of an object in kilogramsInputMax: Saturn V payload
 * The unit is: Kilogram
 */
class Weight
{
public:
  /*!
   * \brief constant defining the minimum valid Weight value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Weight value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Weight value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Weight is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Weight()
    : mWeight(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_WEIGHT_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_WEIGHT_EXPLICIT_CONVERSION_ Weight(double const iWeight)
    : mWeight(iWeight)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Weight(const Weight &other) = default;

  /*!
   * \brief standard move constructor
   */
  Weight(Weight &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Weight
   *
   * \returns Reference to this Weight.
   */
  Weight &operator=(const Weight &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Weight
   *
   * \returns Reference to this Weight.
   */
  Weight &operator=(Weight &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Weight
   *
   * \returns \c true if both Weight are valid and can be taken as numerically equal
   */
  bool operator==(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mWeight - other.mWeight) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Weight.
   *
   * \returns \c true if one of the Weight is not valid or they can be taken as numerically different
   */
  bool operator!=(const Weight &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Weight.
   *
   * \returns \c true if both Weight are valid and
   *   this Weight is strictly numerically greater than other.
   * \note the precision of Weight is considered
   */
  bool operator>(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mWeight > other.mWeight) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Weight.
   *
   * \returns \c true if both Weight are valid and
   *   this Weight is strictly numerically smaller than other.
   * \note the precision of Weight is considered
   */
  bool operator<(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mWeight < other.mWeight) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Weight.
   *
   * \returns \c true if both Weight are valid and
   *   this Weight is numerically greater than other.
   * \note the precision of Weight is considered
   */
  bool operator>=(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mWeight > other.mWeight) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Weight
   *
   * \returns \c true if both Weight are valid and
   *   this Weight is numerically smaller than other.
   * \note the precision of Weight is considered
   */
  bool operator<=(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mWeight < other.mWeight) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Weight
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Weight operator+(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    Weight const result(mWeight + other.mWeight);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Weight
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Weight &operator+=(const Weight &other)
  {
    ensureValid();
    other.ensureValid();
    mWeight += other.mWeight;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Weight
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Weight operator-(const Weight &other) const
  {
    ensureValid();
    other.ensureValid();
    Weight const result(mWeight - other.mWeight);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Weight
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Weight operator-=(const Weight &other)
  {
    ensureValid();
    other.ensureValid();
    mWeight -= other.mWeight;
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
  Weight operator*(const double &scalar) const
  {
    ensureValid();
    Weight const result(mWeight * scalar);
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
  Weight operator/(const double &scalar) const
  {
    Weight const scalarWeight(scalar);
    Weight const result(operator/(scalarWeight));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Weight
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Weight is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Weight &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mWeight / other.mWeight;
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
  Weight operator-() const
  {
    ensureValid();
    Weight const result(-mWeight);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_WEIGHT_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_WEIGHT_EXPLICIT_CONVERSION_ operator double() const
  {
    return mWeight;
  }

  /*!
   * \returns \c true if the Weight in a valid range
   *
   * An Weight value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mWeight);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mWeight) && (mWeight <= cMaxValue);
  }

  /*!
   * \brief ensure that the Weight is valid
   *
   * Throws an std::out_of_range() exception if the Weight
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::Weight)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_WEIGHT_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Weight value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Weight is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Weight
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Weight(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::Weight)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_WEIGHT_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Weight value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Weight (i.e. \ref cMinValue)
   */
  static Weight getMin()
  {
    return Weight(cMinValue);
  }

  /*!
   * \brief get maximum valid Weight (i.e. \ref cMaxValue)
   */
  static Weight getMax()
  {
    return Weight(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Weight (i.e. \ref cPrecisionValue)
   */
  static Weight getPrecision()
  {
    return Weight(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mWeight;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Weight as double value
 * \param[in] value Weight value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::Weight operator*(const double &other, ::ad::physics::Weight const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Weight
 */
inline ::ad::physics::Weight fabs(const ::ad::physics::Weight other)
{
  ::ad::physics::Weight const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Weight
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Weight>::lowest()  (\see Weight::getMin())
 * std::numeric_limits<Weight>::max()  (\see Weight::getMax())
 * std::numeric_limits<Weight>::epsilon()  (\see Weight::getPrecision())
 */
template <> class numeric_limits<::ad::physics::Weight> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::Weight lowest()
  {
    return ::ad::physics::Weight::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::Weight max()
  {
    return ::ad::physics::Weight::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::Weight epsilon()
  {
    return ::ad::physics::Weight::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_WEIGHT
#define GEN_GUARD_AD_PHYSICS_WEIGHT
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
 * \param[in] _value Weight value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Weight const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Weight
 */
inline std::string to_string(::ad::physics::Weight const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_WEIGHT
