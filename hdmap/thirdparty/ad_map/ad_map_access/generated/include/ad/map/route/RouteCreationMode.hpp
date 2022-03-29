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
 * @brief namespace route
 *
 * Handling of routes
 */
namespace route {

/*!
 * \brief DataType RouteCreationMode
 *
 * The mode on how the full route is created
 */
enum class RouteCreationMode : int32_t
{
  /*!
   * undefined value
   */
  Undefined = 0,

  /*!
   * Only add lanes having the same driving direction to the route
   */
  SameDrivingDirection = 1,

  /*!
   * include also lanes in opposite driving direction
   */
  AllRoutableLanes = 2,

  /*!
   * include all neighbors, which might include not explicitly routeable shoulder lanes
   */
  AllNeighborLanes = 3
};

} // namespace route
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::route::RouteCreationMode to std::string helper.
 */
std::string toString(::ad::map::route::RouteCreationMode const e);

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
template <>::ad::map::route::RouteCreationMode fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_ROUTE_ROUTECREATIONMODE
#define GEN_GUARD_AD_MAP_ROUTE_ROUTECREATIONMODE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace route
 *
 * Handling of routes
 */
namespace route {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] value RouteCreationMode value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, RouteCreationMode const &value)
{
  return os << toString(value);
}

} // namespace route
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for RouteCreationMode
 */
inline std::string to_string(::ad::map::route::RouteCreationMode const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_ROUTE_ROUTECREATIONMODE
