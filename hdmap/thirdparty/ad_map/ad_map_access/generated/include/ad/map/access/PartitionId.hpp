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
#include <cstdint>
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
 * @brief namespace access
 *
 * Accessing map data
 */
namespace access {

/*!
 * \brief Define to indicate whether throwing exceptions is enabled
 */
#define AD_MAP_ACCESS_PARTITIONID_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
* \brief Enable/Disable explicit conversion. Currently set to "only explicit conversion".
*/
#define _AD_MAP_ACCESS_PARTITIONID_EXPLICIT_CONVERSION_ explicit
#else
/*!
* \brief Enable/Disable explicit conversion. Currently set to "implicit conversion allowed".
*/
#define _AD_MAP_ACCESS_PARTITIONID_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType PartitionId
 *
 * Defines the id of an partition of the map.
 * The unit is: Identifier
 */
class PartitionId
{
public:
  /*!
   * \brief constant defining the minimum valid PartitionId value (used in isValid())
   */
  static const uint64_t cMinValue;

  /*!
   * \brief constant defining the maximum valid PartitionId value (used in isValid())
   */
  static const uint64_t cMaxValue;

  /*!
   * \brief default constructor
   *
   * The default value of PartitionId is:
   * std::numeric_limits<uint64_t>::quiet_NaN()
   */
  PartitionId()
    : mPartitionId(std::numeric_limits<uint64_t>::quiet_NaN())
  {
  }

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_ACCESS_PARTITIONID_EXPLICIT_CONVERSION\_ defines, if only an explicit conversion is allowed.
   */
  _AD_MAP_ACCESS_PARTITIONID_EXPLICIT_CONVERSION_ PartitionId(uint64_t const iPartitionId)
    : mPartitionId(iPartitionId)
  {
  }

  /*!
   * \brief standard copy constructor
   */
  PartitionId(const PartitionId &other) = default;

  /*!
   * \brief standard move constructor
   */
  PartitionId(PartitionId &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns Reference to this PartitionId.
   */
  PartitionId &operator=(const PartitionId &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns Reference to this PartitionId.
   */
  PartitionId &operator=(PartitionId &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns \c true if both PartitionId are valid and can be taken as numerically equal
   */
  bool operator==(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    return mPartitionId == other.mPartitionId;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PartitionId.
   *
   * \returns \c true if one of the PartitionId is not valid or they can be taken as numerically different
   */
  bool operator!=(const PartitionId &other) const
  {
    return !operator==(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PartitionId.
   *
   * \returns \c true if both PartitionId are valid and
   *   this PartitionId is strictly numerically greater than other.
   * \note the precision of PartitionId is considered
   */
  bool operator>(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mPartitionId > other.mPartitionId) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PartitionId.
   *
   * \returns \c true if both PartitionId are valid and
   *   this PartitionId is strictly numerically smaller than other.
   * \note the precision of PartitionId is considered
   */
  bool operator<(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    return (mPartitionId < other.mPartitionId) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PartitionId.
   *
   * \returns \c true if both PartitionId are valid and
   *   this PartitionId is numerically greater than other.
   * \note the precision of PartitionId is considered
   */
  bool operator>=(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mPartitionId > other.mPartitionId) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns \c true if both PartitionId are valid and
   *   this PartitionId is numerically smaller than other.
   * \note the precision of PartitionId is considered
   */
  bool operator<=(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    return ((mPartitionId < other.mPartitionId) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  PartitionId operator+(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    PartitionId const result(mPartitionId + other.mPartitionId);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  PartitionId &operator+=(const PartitionId &other)
  {
    ensureValid();
    other.ensureValid();
    mPartitionId += other.mPartitionId;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  PartitionId operator-(const PartitionId &other) const
  {
    ensureValid();
    other.ensureValid();
    PartitionId const result(mPartitionId - other.mPartitionId);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other PartitionId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or the result of
   *   the operation is not valid
   */
  PartitionId operator-=(const PartitionId &other)
  {
    ensureValid();
    other.ensureValid();
    mPartitionId -= other.mPartitionId;
    ensureValid();
    return *this;
  }

  /*!
   * \brief conversion to base type: uint64_t
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_ACCESS_PARTITIONID_EXPLICIT_CONVERSION\_ defines, if only explicit calls are allowed.
   */
  _AD_MAP_ACCESS_PARTITIONID_EXPLICIT_CONVERSION_ operator uint64_t() const
  {
    return mPartitionId;
  }

  /*!
   * \returns \c true if the PartitionId in a valid range
   *
   * An PartitionId value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const
  {
    auto const valueClass = std::fpclassify(mPartitionId);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) && (cMinValue <= mPartitionId)
      && (mPartitionId <= cMaxValue);
  }

  /*!
   * \brief ensure that the PartitionId is valid
   *
   * Throws an std::out_of_range() exception if the PartitionId
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const
  {
    if (!isValid())
    {
      spdlog::info("ensureValid(::ad::map::access::PartitionId)>> {} value out of range", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_ACCESS_PARTITIONID_THROWS_EXCEPTION == 1)
      throw std::out_of_range("PartitionId value out of range"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the PartitionId is valid and non zero
   *
   * Throws an std::out_of_range() exception if the PartitionId
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const
  {
    ensureValid();
    if (operator==(PartitionId(0))) // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::map::access::PartitionId)>> {} value is zero", *this); // LCOV_EXCL_BR_LINE
#if (AD_MAP_ACCESS_PARTITIONID_THROWS_EXCEPTION == 1)
      throw std::out_of_range("PartitionId value is zero"); // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid PartitionId (i.e. \ref cMinValue)
   */
  static PartitionId getMin()
  {
    return PartitionId(cMinValue);
  }

  /*!
   * \brief get maximum valid PartitionId (i.e. \ref cMaxValue)
   */
  static PartitionId getMax()
  {
    return PartitionId(cMaxValue);
  }

private:
  /*!
   * \brief the actual value of the type
   */
  uint64_t mPartitionId;
};

} // namespace access
} // namespace map
} // namespace ad
/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief specialization of the std::numeric_limits for PartitionId
 *
 * Derived from std::numeric_limits<uint64_t> with overloaded functions:
 * std::numeric_limits<PartitionId>::lowest()  (\see PartitionId::getMin())
 * std::numeric_limits<PartitionId>::max()  (\see PartitionId::getMax())
 * std::numeric_limits<PartitionId>::epsilon()  (\see PartitionId::getPrecision())
 */
template <> class numeric_limits<::ad::map::access::PartitionId> : public numeric_limits<uint64_t>
{
public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::access::PartitionId lowest()
  {
    return ::ad::map::access::PartitionId::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::access::PartitionId max()
  {
    return ::ad::map::access::PartitionId::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::access::PartitionId epsilon()
  {
    return ::ad::map::access::PartitionId(0);
  }
};

} // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_ACCESS_PARTITIONID
#define GEN_GUARD_AD_MAP_ACCESS_PARTITIONID
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace access
 *
 * Accessing map data
 */
namespace access {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value PartitionId value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, PartitionId const &_value)
{
  return os << uint64_t(_value);
}

} // namespace access
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for PartitionId
 */
inline std::string to_string(::ad::map::access::PartitionId const &value)
{
  return to_string(static_cast<uint64_t>(value));
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_ACCESS_PARTITIONID
