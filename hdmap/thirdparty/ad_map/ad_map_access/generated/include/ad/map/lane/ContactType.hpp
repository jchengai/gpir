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
#include <memory>
#include <string>
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
 * \brief DataType ContactType
 *
 * Type of the contact between two objects.
 */
enum class ContactType : int32_t {
  /*!
   * Invalid value.
   */
  INVALID = 0,

  /*!
   * Unknown value.
   */
  UNKNOWN = 1,

  /*!
   * There is no physical or legal obstacles between two objects.
   */
  FREE = 2,

  /*!
   * Transition between one lane to direct lateral neighboor.
   */
  LANE_CHANGE = 3,

  /*!
   * Transition between one lane to longitudinal direct neighboor.
   */
  LANE_CONTINUATION = 4,

  /*!
   * End of the Lane - line.
   */
  LANE_END = 5,

  /*!
   * End of the Lane - point.
   */
  SINGLE_POINT = 6,

  /*!
   * STOP regulation at the end of the lane.
   */
  STOP = 7,

  /*!
   * STOP 3-way, 4-way etc regulation at the end of the lane.
   */
  STOP_ALL = 8,

  /*!
   * YIELD regulation at the end of the lane.
   */
  YIELD = 9,

  /*!
   * Gate with barrier at the end of the lane.
   */
  GATE_BARRIER = 10,

  /*!
   * Tolbooth barrier at the end of the lane.
   */
  GATE_TOLBOOTH = 11,

  /*!
   * Spikes (in direction) at the end of the lane.
   */
  GATE_SPIKES = 12,

  /*!
   * Spikes (in opposite direction) at the end of the lane.
   */
  GATE_SPIKES_CONTRA = 13,

  /*!
   * Curb up at the object side.
   */
  CURB_UP = 14,

  /*!
   * Curb down at the object.
   */
  CURB_DOWN = 15,

  /*!
   * Speed bump at the end of the lane.
   */
  SPEED_BUMP = 16,

  /*!
   * Traffic light at the end of the lane.
   */
  TRAFFIC_LIGHT = 17,

  /*!
   * Crosswalk at the end of the lane.
   */
  CROSSWALK = 18,

  /*!
   * Priority to the right regulation at the end of the lane.
   */
  PRIO_TO_RIGHT = 19,

  /*!
   * Right of way regulation at the end of the lane.
   */
  RIGHT_OF_WAY = 20,

  /*!
   * Priority to right and to straight regulation at the end of the lane
   * (Singapore).
   */
  PRIO_TO_RIGHT_AND_STRAIGHT = 21
};

}  // namespace lane
}  // namespace map
}  // namespace ad
/*!
 * \brief Conversion of ::ad::map::lane::ContactType to std::string helper.
 */
std::string toString(::ad::map::lane::ContactType const e);

/*!
 * \brief Conversion from std::string to enum type T helper.
 *
 * \param [in] str - a fully qualified string name of enum class type
 *
 * \return T enum value
 *
 * \throws std::out_of_range exception if the given string does not match any
 * enum type
 *
 * Example usage:
 * \code
 *   auto value = fromString<SomeEnumType>("SomeEnumType::eValue");
 *   assert(value == SomeEnumType::eValue);
 *   // Or:
 *   auto value = fromString<SomeEnumType>("eValue");
 *   assert(value == SomeEnumType::eValue);
 * \endcode
 */
template <typename EnumType>
EnumType fromString(std::string const &str);

/*!
 * \brief Conversion from std::string to enum type T helper.
 *
 * \param [in] str - a fully qualified string name of enum class type
 *
 * \return T enum value
 *
 * \throws std::out_of_range exception if the given string does not match any
 * enum type
 *
 * Example usage:
 * \code
 *   auto value = fromString<SomeEnumType>("SomeEnumType::eValue");
 *   assert(value == SomeEnumType::eValue);
 *   // Or:
 *   auto value = fromString<SomeEnumType>("eValue");
 *   assert(value == SomeEnumType::eValue);
 * \endcode
 */
template <>
::ad::map::lane::ContactType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_CONTACTTYPE
#define GEN_GUARD_AD_MAP_LANE_CONTACTTYPE
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
 * \param[in] value ContactType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ContactType const &value) {
  return os << toString(value);
}

}  // namespace lane
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ContactType
 */
inline std::string to_string(::ad::map::lane::ContactType const &value) {
  return ::toString(value);
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_LANE_CONTACTTYPE
