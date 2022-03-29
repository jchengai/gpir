// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

/**
 * @file
 */

#pragma once

#include "ad/physics/AccelerationRangeValidInputRange.hpp"
#include "ad/physics/MetricRangeValidInputRange.hpp"
#include "ad/physics/ParametricRangeValidInputRange.hpp"
#include "ad/physics/SpeedRangeValidInputRange.hpp"

namespace ad {
namespace physics {
/**
 * @param[in] range range object.
 * @return true if Range is valid.
 */
template <typename RangeType> bool isRangeValid(RangeType const &range)
{
  return withinValidInputRange(range);
}

/**
 * @param[in] range range object.
 * @return true if Range is valid but empty.
 */
template <typename RangeType> bool isRangeEmpty(RangeType const &range)
{
  return isRangeValid(range) && (range.minimum == range.maximum);
}

/**
 * @brief Checks if value of RangeBaseType is within range.
 * @param[in] range range object.
 * @param[in] value object of range base type.
 * @return true if RangeBaseType is within range.
 */
template <typename RangeType, typename RangeBaseType>
bool isWithinRange(RangeType const &range, RangeBaseType const &value)
{
  return (range.minimum <= value) && (value <= range.maximum);
}

/**
 * @brief Checks if right range is contained in left one.
 * @param[in] left range object.
 * @param[in] right range object.
 * @return true if right range is contained in left one.
 */
template <typename RangeType> bool isRangeContained(RangeType const &left, RangeType const &right)
{
  return (left.minimum <= right.minimum) && (right.maximum <= left.maximum);
}

/**
 * @brief Checks left range overlaps right one.
 * @param[in] left range object.
 * @param[in] right range object.
 * @return true if left range overlaps right one.
 */
template <typename RangeType> bool doRangesOverlap(RangeType const &left, RangeType const &right)
{
  return isWithinRange(left, right.minimum) || isWithinRange(left, right.maximum) || isWithinRange(right, left.minimum)
    || isWithinRange(right, left.maximum) || isRangeContained(left, right) || isRangeContained(right, left);
}

/**
 * @brief Checks left range is disjunct from right one.
 * @param[in] left range object.
 * @param[in] right range object.
 * @return true if this range is disjunct from right one.
 */
template <typename RangeType> bool areRangesDisjunct(RangeType const &left, RangeType const &right)
{
  return !doRangesOverlaps(left, right);
}

/**
 * @brief Checks if left range comes directly before right one.
 * @param[in] left range object.
 * @param[in] right range object.
 * @return true if left range comes directly before right one.
 */
template <typename RangeType> bool doesRangePredate(RangeType const &left, RangeType const &right)
{
  return left.maximum == right.minimum;
}

/**
 * @brief Checks if left range comes directly after right one.
 * @param[in] left range object.
 * @param[in] right range object.
 * @return true if left range comes directly after right one.
 */
template <typename RangeType> bool doesRangeSucceed(RangeType const &left, RangeType const &right)
{
  return left.minimum == right.maximum;
}

/**
 * @brief Extends left range with right one.
 *        Direction of extension depends of the relative position of ranges.
 * @param[in] left range object.
 * @param[in] right range object.
 * @return true if successful. false indicates that right range neither Predates
 *         nor Suceeds left one.
 */
template <typename RangeType> bool extendRangeWith(RangeType &left, RangeType const &right)
{
  if (doesRangePredate(left, right))
  {
    left.maximum = right.maximum;
    return true;
  }
  else if (doesRangeSucceed(left, right))
  {
    left.minimum = right.minimum;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Calculate union of left and right range.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns Union of left and right range.
 */
template <typename RangeType> RangeType getUnionRange(RangeType const &left, RangeType const &right)
{
  RangeType result;
  result.minimum = std::min(left.minimum, right.minimum);
  result.maximum = std::max(left.maximum, right.maximum);
  return result;
}

/**
 * @brief Calculate union of left range with additional value
 * @param[in] left range object.
 * @param[in] value value type object.
 * @returns Union of left range value.
 */
template <typename RangeType, typename RangeBaseType>
RangeType getUnionRange(RangeType const &left, RangeBaseType const &value)
{
  RangeType result;
  result.minimum = std::min(left.minimum, value);
  result.maximum = std::max(left.maximum, value);
  return result;
}

/**
 * @returns store union of right and left range into left range.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns true if successful.
 */
template <typename RangeType> bool unionRangeWith(RangeType &left, RangeType const &right)
{
  left = getUnionRange(left, right);
  return isRangeValid(left);
}

/**
 * @returns store union of left range with additional value.
 * @param[in] left range object.
 * @param[in] value value type object.
 * @returns true if successful.
 */
template <typename RangeType, typename RangeBaseType> bool unionRangeWith(RangeType &left, RangeBaseType const &value)
{
  left = getUnionRange(left, value);
  return isRangeValid(left);
}

/**
 * @brief Intersect this range with right one.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns intersection range.
 * \note If objects are not overlapping, intersection range will be invalid.
 */
template <typename RangeType> RangeType getIntersectionRange(RangeType const &left, RangeType const &right)
{
  RangeType result;
  if (doRangesOverlap(left, right))
  {
    result.minimum = std::max(left.minimum, right.minimum);
    result.maximum = std::min(left.maximum, right.maximum);
  }
  return result;
}

/**
 * @brief Intersect left range with right one and store intersection in left.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns true if, after intersection, left object is valid.
 * \note If objects are not overlapping, left object will become invalid.
 */
template <typename RangeType> bool intersectRangeWith(RangeType &left, RangeType const &right)
{
  left = getIntersectionRange(left, right);
  return isRangeValid(left);
}

} // namespace physics
} // namespace ad

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>(ad::physics::ParametricRange const &left, ad::physics::ParametricRange const &right)
{
  return left.minimum > right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>=(ad::physics::ParametricRange const &left, ad::physics::ParametricRange const &right)
{
  return left.minimum >= right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<(ad::physics::ParametricRange const &left, ad::physics::ParametricRange const &right)
{
  return left.maximum < right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<=(ad::physics::ParametricRange const &left, ad::physics::ParametricRange const &right)
{
  return left.maximum <= right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>(ad::physics::MetricRange const &left, ad::physics::MetricRange const &right)
{
  return left.minimum > right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>=(ad::physics::MetricRange const &left, ad::physics::MetricRange const &right)
{
  return left.minimum >= right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<(ad::physics::MetricRange const &left, ad::physics::MetricRange const &right)
{
  return left.maximum < right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<=(ad::physics::MetricRange const &left, ad::physics::MetricRange const &right)
{
  return left.maximum <= right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>(ad::physics::AccelerationRange const &left, ad::physics::AccelerationRange const &right)
{
  return left.minimum > right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>=(ad::physics::AccelerationRange const &left, ad::physics::AccelerationRange const &right)
{
  return left.minimum >= right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<(ad::physics::AccelerationRange const &left, ad::physics::AccelerationRange const &right)
{
  return left.maximum < right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<=(ad::physics::AccelerationRange const &left, ad::physics::AccelerationRange const &right)
{
  return left.maximum <= right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>(ad::physics::SpeedRange const &left, ad::physics::SpeedRange const &right)
{
  return left.minimum > right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly after right range.
 */
inline bool operator>=(ad::physics::SpeedRange const &left, ad::physics::SpeedRange const &right)
{
  return left.minimum >= right.maximum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<(ad::physics::SpeedRange const &left, ad::physics::SpeedRange const &right)
{
  return left.maximum < right.minimum;
}

/**
 * @brief Comparison operator for range based types.
 * @param[in] left range object.
 * @param[in] right range object.
 * @returns True if left range comes strictly before right range.
 */
inline bool operator<=(ad::physics::SpeedRange const &left, ad::physics::SpeedRange const &right)
{
  return left.maximum <= right.minimum;
}
