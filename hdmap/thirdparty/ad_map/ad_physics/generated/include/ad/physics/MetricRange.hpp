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

#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include "ad/physics/Distance.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType MetricRange
 *
 * A metric range described by its borders: [minimum, maximum].
 */
struct MetricRange
{
  /*!
   * \brief Smart pointer on MetricRange
   */
  typedef std::shared_ptr<MetricRange> Ptr;

  /*!
   * \brief Smart pointer on constant MetricRange
   */
  typedef std::shared_ptr<MetricRange const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  MetricRange() = default;

  /*!
   * \brief standard destructor
   */
  ~MetricRange() = default;

  /*!
   * \brief standard copy constructor
   */
  MetricRange(const MetricRange &other) = default;

  /*!
   * \brief standard move constructor
   */
  MetricRange(MetricRange &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other MetricRange
   *
   * \returns Reference to this MetricRange.
   */
  MetricRange &operator=(const MetricRange &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other MetricRange
   *
   * \returns Reference to this MetricRange.
   */
  MetricRange &operator=(MetricRange &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other MetricRange
   *
   * \returns \c true if both MetricRange are equal
   */
  bool operator==(const MetricRange &other) const
  {
    return (minimum == other.minimum) && (maximum == other.maximum);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other MetricRange.
   *
   * \returns \c true if both MetricRange are different
   */
  bool operator!=(const MetricRange &other) const
  {
    return !operator==(other);
  }

  /*!
   * The minimum value of the metric range.
   */
  ::ad::physics::Distance minimum{std::numeric_limits<Distance>::lowest()};

  /*!
   * The maximum value of the metric range.
   */
  ::ad::physics::Distance maximum{std::numeric_limits<Distance>::max()};
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_METRICRANGE
#define GEN_GUARD_AD_PHYSICS_METRICRANGE
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
 * \param[in] _value MetricRange value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, MetricRange const &_value)
{
  os << "MetricRange(";
  os << "minimum:";
  os << _value.minimum;
  os << ",";
  os << "maximum:";
  os << _value.maximum;
  os << ")";
  return os;
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for MetricRange
 */
inline std::string to_string(::ad::physics::MetricRange const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_METRICRANGE
