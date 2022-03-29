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
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/*!
 * \brief Define to indicate whether throwing exceptions is enabled
 */
#define AD_MAP_LANE_LANEID_THROWS_EXCEPTION 1

#if SAFE_DATATYPES_EXPLICIT_CONVERSION
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "only explicit
 * conversion".
 */
#define _AD_MAP_LANE_LANEID_EXPLICIT_CONVERSION_ explicit
#else
/*!
 * \brief Enable/Disable explicit conversion. Currently set to "implicit
 * conversion allowed".
 */
#define _AD_MAP_LANE_LANEID_EXPLICIT_CONVERSION_
#endif

/*!
 * \brief DataType LaneId
 *
 * Defines the identifier of a lane of the map.
 * The unit is: Identifier
 */
class LaneId {
 public:
  /*!
   * \brief constant defining the minimum valid LaneId value (used in isValid())
   */
  static const uint64_t cMinValue;

  /*!
   * \brief constant defining the maximum valid LaneId value (used in isValid())
   */
  static const uint64_t cMaxValue;

  /*!
   * \brief default constructor
   *
   * The default value of LaneId is:
   * std::numeric_limits<uint64_t>::quiet_NaN()
   */
  LaneId() : mLaneId(std::numeric_limits<uint64_t>::quiet_NaN()) {}

  /*!
   * \brief standard constructor
   *
   * \note \ref \_AD_MAP_LANE_LANEID_EXPLICIT_CONVERSION\_ defines, if only an
   * explicit conversion is allowed.
   */
  _AD_MAP_LANE_LANEID_EXPLICIT_CONVERSION_ LaneId(uint64_t const iLaneId)
      : mLaneId(iLaneId) {}

  /*!
   * \brief standard copy constructor
   */
  LaneId(const LaneId &other) = default;

  /*!
   * \brief standard move constructor
   */
  LaneId(LaneId &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other LaneId
   *
   * \returns Reference to this LaneId.
   */
  LaneId &operator=(const LaneId &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other LaneId
   *
   * \returns Reference to this LaneId.
   */
  LaneId &operator=(LaneId &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneId
   *
   * \returns \c true if both LaneId are valid and can be taken as numerically
   * equal
   */
  bool operator==(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    return mLaneId == other.mLaneId;
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneId.
   *
   * \returns \c true if one of the LaneId is not valid or they can be taken as
   * numerically different
   */
  bool operator!=(const LaneId &other) const { return !operator==(other); }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneId.
   *
   * \returns \c true if both LaneId are valid and
   *   this LaneId is strictly numerically greater than other.
   * \note the precision of LaneId is considered
   */
  bool operator>(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    return (mLaneId > other.mLaneId) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneId.
   *
   * \returns \c true if both LaneId are valid and
   *   this LaneId is strictly numerically smaller than other.
   * \note the precision of LaneId is considered
   */
  bool operator<(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    return (mLaneId < other.mLaneId) && operator!=(other);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneId.
   *
   * \returns \c true if both LaneId are valid and
   *   this LaneId is numerically greater than other.
   * \note the precision of LaneId is considered
   */
  bool operator>=(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    return ((mLaneId > other.mLaneId) || operator==(other));
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneId
   *
   * \returns \c true if both LaneId are valid and
   *   this LaneId is numerically smaller than other.
   * \note the precision of LaneId is considered
   */
  bool operator<=(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    return ((mLaneId < other.mLaneId) || operator==(other));
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other LaneId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  LaneId operator+(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    LaneId const result(mLaneId + other.mLaneId);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other LaneId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  LaneId &operator+=(const LaneId &other) {
    ensureValid();
    other.ensureValid();
    mLaneId += other.mLaneId;
    ensureValid();
    return *this;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other LaneId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  LaneId operator-(const LaneId &other) const {
    ensureValid();
    other.ensureValid();
    LaneId const result(mLaneId - other.mLaneId);
    result.ensureValid();
    return result;
  }

  /**
   * \brief standard arithmetic operator
   *
   * \param[in] other Other LaneId
   *
   * \returns Result of arithmetic operation.
   *
   * \note throws a std::out_of_range exception if one of the two operands or
   * the result of the operation is not valid
   */
  LaneId operator-=(const LaneId &other) {
    ensureValid();
    other.ensureValid();
    mLaneId -= other.mLaneId;
    ensureValid();
    return *this;
  }

  /*!
   * \brief conversion to base type: uint64_t
   *
   * \note the conversion to the base type removes the physical unit.
   *       \ref \_AD_MAP_LANE_LANEID_EXPLICIT_CONVERSION\_ defines, if only
   * explicit calls are allowed.
   */
  _AD_MAP_LANE_LANEID_EXPLICIT_CONVERSION_ operator uint64_t() const {
    return mLaneId;
  }

  /*!
   * \returns \c true if the LaneId in a valid range
   *
   * An LaneId value is defined to be valid if:
   * - It is normal or zero (see std::fpclassify())
   * - \ref cMinValue <= value <= \ref cMaxValue
   */
  bool isValid() const {
    auto const valueClass = std::fpclassify(mLaneId);
    return ((valueClass == FP_NORMAL) || (valueClass == FP_ZERO)) &&
           (cMinValue <= mLaneId) && (mLaneId <= cMaxValue);
  }

  /*!
   * \brief ensure that the LaneId is valid
   *
   * Throws an std::out_of_range() exception if the LaneId
   * in not valid (i.e. isValid() returns false)
   */
  void ensureValid() const {
    if (!isValid()) {
      spdlog::info(
          "ensureValid(::ad::map::lane::LaneId)>> {} value out of range",
          *this);  // LCOV_EXCL_BR_LINE
#if (AD_MAP_LANE_LANEID_THROWS_EXCEPTION == 1)
      throw std::out_of_range(
          "LaneId value out of range");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief ensure that the LaneId is valid and non zero
   *
   * Throws an std::out_of_range() exception if the LaneId
   * in not valid or zero (i.e. isValid() returns false)
   */
  void ensureValidNonZero() const {
    ensureValid();
    if (operator==(LaneId(0)))  // LCOV_EXCL_BR_LINE
    {
      spdlog::info("ensureValid(::ad::map::lane::LaneId)>> {} value is zero",
                   *this);  // LCOV_EXCL_BR_LINE
#if (AD_MAP_LANE_LANEID_THROWS_EXCEPTION == 1)
      throw std::out_of_range("LaneId value is zero");  // LCOV_EXCL_BR_LINE
#endif
    }
  }

  /*!
   * \brief get minimum valid LaneId (i.e. \ref cMinValue)
   */
  static LaneId getMin() { return LaneId(cMinValue); }

  /*!
   * \brief get maximum valid LaneId (i.e. \ref cMaxValue)
   */
  static LaneId getMax() { return LaneId(cMaxValue); }

  uint64_t value() const { return mLaneId; }

 private:
  /*!
   * \brief the actual value of the type
   */
  uint64_t mLaneId;
};

}  // namespace lane
}  // namespace map
}  // namespace ad
/*!
 * \brief namespace std
 */
namespace std {

/*!
 * \brief specialization of the std::numeric_limits for LaneId
 *
 * Derived from std::numeric_limits<uint64_t> with overloaded functions:
 * std::numeric_limits<LaneId>::lowest()  (\see LaneId::getMin())
 * std::numeric_limits<LaneId>::max()  (\see LaneId::getMax())
 * std::numeric_limits<LaneId>::epsilon()  (\see LaneId::getPrecision())
 */
template <>
class numeric_limits<::ad::map::lane::LaneId>
    : public numeric_limits<uint64_t> {
 public:
  /*!
   * \see std::numeric_limits::lowest()
   */
  static inline ::ad::map::lane::LaneId lowest() {
    return ::ad::map::lane::LaneId::getMin();
  }
  /*!
   * \see std::numeric_limits::max()
   */
  static inline ::ad::map::lane::LaneId max() {
    return ::ad::map::lane::LaneId::getMax();
  }

  /*!
   * \see std::numeric_limits::epsilon()
   */
  static inline ::ad::map::lane::LaneId epsilon() {
    return ::ad::map::lane::LaneId(0);
  }
};

}  // namespace std

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_LANEID
#define GEN_GUARD_AD_MAP_LANE_LANEID
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value LaneId value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, LaneId const &_value) {
  return os << uint64_t(_value);
}

}  // namespace lane
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for LaneId
 */
inline std::string to_string(::ad::map::lane::LaneId const &value) {
  return to_string(static_cast<uint64_t>(value));
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_LANE_LANEID
