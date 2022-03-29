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
 * \brief DataType ConnectingRouteType
 *
 * The type of a connecting route.
 */
enum class ConnectingRouteType : int32_t
{
  /*!
   * There exists no connecting route (according to the search length/distance restrictions): both routeA and routeB are
   * empty
   */
  Invalid = 0,

  /*!
   * A and B are driving in same direction: one is able to reach the other by driving
   * forward, but not vice versa
   * If A is in front of B: routeA is empty, routeB contains the route part from B towards
   * A
   * If B is in front of A: routeB is empty, routeA contains the route part from A towards
   * B
   */
  Following = 1,

  /*!
   * A and B driving in opposite direction: both are able to reach each other by driving
   * forward:
   * routeA contains the route part from A towards B and routeB the inverted route part
   * from B towards A.
   */
  Opposing = 2,

  /*!
   * A and B can reach each other directly, but their routes merge at some point:
   * routeA contains the route part from A towards the merge point
   * routeB contains the route part from B towards the merge point
   */
  Merging = 3
};

} // namespace route
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::route::ConnectingRouteType to std::string helper.
 */
std::string toString(::ad::map::route::ConnectingRouteType const e);

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
template <>::ad::map::route::ConnectingRouteType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_ROUTE_CONNECTINGROUTETYPE
#define GEN_GUARD_AD_MAP_ROUTE_CONNECTINGROUTETYPE
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
 * \param[in] value ConnectingRouteType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ConnectingRouteType const &value)
{
  return os << toString(value);
}

} // namespace route
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ConnectingRouteType
 */
inline std::string to_string(::ad::map::route::ConnectingRouteType const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_ROUTE_CONNECTINGROUTETYPE
