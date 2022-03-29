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
 * @brief namespace intersection
 *
 * Contains datatypes related to intersection handling
 */
namespace intersection {

/*!
 * \brief DataType IntersectionType
 *
 * Defines the type of intersection
 */
enum class IntersectionType : int32_t
{
  /*!
   * not set, invalid
   */
  Unknown = 0,

  /*!
   * some call it GiveWay
   */
  Yield = 1,

  /*!
   * STOP in front
   */
  Stop = 2,

  /*!
   * all way stop
   */
  AllWayStop = 3,

  /*!
   * being on the priority road
   */
  HasWay = 4,

  /*!
   * crosswalk
   */
  Crosswalk = 5,

  /*!
   * priority to the right
   */
  PriorityToRight = 6,

  /*!
   * priority to the right and straight (e.g. Singapore)
   */
  PriorityToRightAndStraight = 7,

  /*!
   * hitting a traffic light, the applicable traffic lights are return with
   */
  TrafficLight = 8
};

} // namespace intersection
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::intersection::IntersectionType to std::string helper.
 */
std::string toString(::ad::map::intersection::IntersectionType const e);

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
template <>::ad::map::intersection::IntersectionType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_INTERSECTION_INTERSECTIONTYPE
#define GEN_GUARD_AD_MAP_INTERSECTION_INTERSECTIONTYPE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace intersection
 *
 * Contains datatypes related to intersection handling
 */
namespace intersection {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] value IntersectionType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, IntersectionType const &value)
{
  return os << toString(value);
}

} // namespace intersection
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for IntersectionType
 */
inline std::string to_string(::ad::map::intersection::IntersectionType const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_INTERSECTION_INTERSECTIONTYPE
