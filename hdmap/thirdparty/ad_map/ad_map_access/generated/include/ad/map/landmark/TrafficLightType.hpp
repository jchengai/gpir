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
 * @brief namespace landmark
 *
 * Handling of landmarks
 */
namespace landmark {

/*!
 * \brief DataType TrafficLightType
 *
 * Type of traffic light
 */
enum class TrafficLightType : int32_t
{
  /*!
   * Invalid value.
   */
  INVALID = 0,

  /*!
   * Unknown value.
   */
  UNKNOWN = 1,

  /*!
   * 2 solid lights: R+Y
   */
  SOLID_RED_YELLOW = 2,

  /*!
   * 3 solid lights: R+Y+G
   */
  SOLID_RED_YELLOW_GREEN = 3,

  /*!
   * 3 lights with a turn-left arrow: R+Y+G
   */
  LEFT_RED_YELLOW_GREEN = 4,

  /*!
   * 3 lights with a turn-right arrow: R+Y+G
   */
  RIGHT_RED_YELLOW_GREEN = 5,

  /*!
   * 3 lights with a straight arrow: R+Y+G
   */
  STRAIGHT_RED_YELLOW_GREEN = 6,

  /*!
   * 3 lights with a straight or turn-left arrow: R+Y+G
   */
  LEFT_STRAIGHT_RED_YELLOW_GREEN = 7,

  /*!
   * 3 lights with a straight or turn-right arrow: R+Y+G
   */
  RIGHT_STRAIGHT_RED_YELLOW_GREEN = 8,

  /*!
   * For pedestrians with 2 lights: R+G
   */
  PEDESTRIAN_RED_GREEN = 9,

  /*!
   * For bikes with 2 lights: R+G
   */
  BIKE_RED_GREEN = 10,

  /*!
   * For bikes and pedestrians with 2 lights: R+G
   */
  BIKE_PEDESTRIAN_RED_GREEN = 11,

  /*!
   * For pedestrians with 3 lights: R+Y+G
   */
  PEDESTRIAN_RED_YELLOW_GREEN = 12,

  /*!
   * For bikes with 3 lights: R+Y+G
   */
  BIKE_RED_YELLOW_GREEN = 13,

  /*!
   * For bikes and pedestrians with 3 lights: R+Y+G
   */
  BIKE_PEDESTRIAN_RED_YELLOW_GREEN = 14
};

} // namespace landmark
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::landmark::TrafficLightType to std::string helper.
 */
std::string toString(::ad::map::landmark::TrafficLightType const e);

/*!
 * \brief Conversion from std::string to enum type T helper.
 *
 * \param [in] str - a fully qualified string name of enum class type
 *
 * \return T enum value
 *
 * \throws std::out_of_range exception if the given string does not match any enum type
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
template <typename EnumType> EnumType fromString(std::string const &str);

/*!
 * \brief Conversion from std::string to enum type T helper.
 *
 * \param [in] str - a fully qualified string name of enum class type
 *
 * \return T enum value
 *
 * \throws std::out_of_range exception if the given string does not match any enum type
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
template <>::ad::map::landmark::TrafficLightType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANDMARK_TRAFFICLIGHTTYPE
#define GEN_GUARD_AD_MAP_LANDMARK_TRAFFICLIGHTTYPE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace landmark
 *
 * Handling of landmarks
 */
namespace landmark {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] value TrafficLightType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, TrafficLightType const &value)
{
  return os << toString(value);
}

} // namespace landmark
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for TrafficLightType
 */
inline std::string to_string(::ad::map::landmark::TrafficLightType const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANDMARK_TRAFFICLIGHTTYPE
