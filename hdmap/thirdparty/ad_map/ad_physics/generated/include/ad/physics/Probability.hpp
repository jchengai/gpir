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
#define AD_PHYSICS_PROBABILITY_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_PHYSICS_PROBABILITY_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_PHYSICS_PROBABILITY_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType Probability
 *
 * A probability value in the range of [0.0; 1.0].
 */
class Probability
{
public:
  /*!
   * \brief constant defining the minimum valid Probability value (used in isValid())
   */
  static const double cMinValue;

  /*!
   * \brief constant defining the maximum valid Probability value (used in isValid())
   */
  static const double cMaxValue;

  /*!
   * \brief constant defining the assumed Probability value accuracy
   *   (used in comparison operator==(), operator!=())
   */
  static const double cPrecisionValue;

  /*!
   * \brief default constructor
   *
   * The default value of Probability is:
   * std::numeric_limits<double>::quiet_NaN()
   */
  Probability()
    : mProbability(std::numeric_limits<double>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_PHYSICS_PROBABILITY_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_PHYSICS_PROBABILITY_EXPLICIT_CONVERSION_ Probability(double const iProbability)
    : mProbability(iProbability)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  Probability(const Probability &other) = default;

  /*!
   * \brief standard move constructor
   */
  Probability(Probability &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Probability
   *
   * \returns Reference to this Probability.
   */
  Probability &operator=(const Probability &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Probability
   *
   * \returns Reference to this Probability.
   */
  Probability &operator=(Probability &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Probability
   *
   * \returns \c true if both Probability are valid and can be taken as numerically equal
   */
  bool operator==(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    return std::fabs(mProbability - other.mProbability) < cPrecisionValue;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Probability.
   *
   * \returns \c true if one of the Probability is not valid or they can be taken as numerically different
   */
  bool operator!=(const Probability &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Probability.
   *
   * \returns \c true if both Probability are valid and
   *   this Probability is strictly numerically greater than other.
   * \note the precision of Probability is considered
   */
  bool operator>(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mProbability > other.mProbability) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Probability.
   *
   * \returns \c true if both Probability are valid and
   *   this Probability is strictly numerically smaller than other.
   * \note the precision of Probability is considered
   */
  bool operator<(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mProbability < other.mProbability) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Probability.
   *
   * \returns \c true if both Probability are valid and
   *   this Probability is numerically greater than other.
   * \note the precision of Probability is considered
   */
  bool operator>=(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mProbability > other.mProbability) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Probability
   *
   * \returns \c true if both Probability are valid and
   *   this Probability is numerically smaller than other.
   * \note the precision of Probability is considered
   */
  bool operator<=(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mProbability < other.mProbability) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Probability
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Probability operator+(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    Probability const result(mProbability + other.mProbability);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Probability
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Probability &operator+=(const Probability &other)
  {
    ensureValid();
    other.ensureValid();
    mProbability += other.mProbability;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Probability
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Probability operator-(const Probability &other) const
  {
    ensureValid();
    other.ensureValid();
    Probability const result(mProbability - other.mProbability);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Probability
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  Probability operator-=(const Probability &other)
  {
    ensureValid();
    other.ensureValid();
    mProbability -= other.mProbability;
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
  Probability operator*(const double &scalar) const
  {
    ensureValid();
    Probability const result(mProbability * scalar);
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
  Probability operator/(const double &scalar) const
  {
    Probability const scalarProbability(scalar);
    Probability const result(operator/(scalarProbability));
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other Probability
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid or other is zero
   * \note since Probability is a type with physical unit, the division results in the dimensionless type.
   */
  double operator/(const Probability &other) const
  {
    ensureValid();
    other.ensureValidNonZero();
    double const result = mProbability / other.mProbability;
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
  Probability operator-() const
  {
    ensureValid();
    Probability const result(-mProbability);
    result.ensureValid(); // LCOV_EXCL_BR_LINE Some types do not throw an exception
    return result;
  }

  /*!
   * \brief conversion to base type: double
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_PHYSICS_PROBABILITY_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_PHYSICS_PROBABILITY_EXPLICIT_CONVERSION_ operator double() const
  {
    return mProbability;
  }

  /*!
   * \returns \c true if the Probability in a valid range
   *
   * An Probability value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mProbability);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mProbability)
      && (mProbability <= cMaxValue);
  }

  /*!
   * \brief ensure that the Probability is valid
   *
   * Throws an std::out_of_range() exception if the Probability
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::physics::Probability)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_PROBABILITY_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Probability value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the Probability is valid and non zero
   *
   * Throws an std::out_of_range() exception if the Probability
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(Probability(0.))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::physics::Probability)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_PHYSICS_PROBABILITY_THROWS_EXCEPTION == 1)
      throw std::out_of_range("Probability value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid Probability (i.e. \ref cMinValue)
   */
  static Probability getMin()
  {
    return Probability(cMinValue);
  }

  /*!
   * \brief get maximum valid Probability (i.e. \ref cMaxValue)
   */
  static Probability getMax()
  {
    return Probability(cMaxValue);
  }

  /*!
   * \brief get assumed accuracy of Probability (i.e. \ref cPrecisionValue)
   */
  static Probability getPrecision()
  {
    return Probability(cPrecisionValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  double mProbability;
};

} // namespace physics
} // namespace ad
/**
 * \brief standard arithmetic operator
 *
 * \param[in] other Other Probability as double value
 * \param[in] value Probability value
 *
 * \returns Result of arithmetic operation.
 *
 * \note throws a std::out_of_range exception if \c value or the result of
 *   the operation is not valid
 */
inline ::ad::physics::Probability operator*(const double &other, ::ad::physics::Probability const &value)
{
  return value.operator*(other);
}

/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief overload of the std::fabs for Probability
 */
inline ::ad::physics::Probability fabs(const ::ad::physics::Probability other)
{
  ::ad::physics::Probability const result(std::fabs(static_cast<double>(other)));
  return result;
}

/*!
 * \brief specialization of the std::numeric_limits for Probability
 *
 * Derived from std::numeric_limits<double> with overloaded functions:
 * std::numeric_limits<Probability>::lowest()  (\see Probability::getMin())
 * std::numeric_limits<Probability>::max()  (\see Probability::getMax())
 * std::numeric_limits<Probability>::epsilon()  (\see Probability::getPrecision())
 */
template <> class numeric_limits<::ad::physics::Probability> : public numeric_limits<double>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::physics::Probability lowest()
  {
    return ::ad::physics::Probability::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::physics::Probability max()
  {
    return ::ad::physics::Probability::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::physics::Probability epsilon()
  {
    return ::ad::physics::Probability::getPrecision();
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_PROBABILITY
#define GEN_GUARD_AD_PHYSICS_PROBABILITY
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
 * \param[in] _value Probability value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Probability const &_value)
{
  return os << double(_value);
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Probability
 */
inline std::string to_string(::ad::physics::Probability const &value)
{
  return to_string(static_cast<double>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_PROBABILITY
