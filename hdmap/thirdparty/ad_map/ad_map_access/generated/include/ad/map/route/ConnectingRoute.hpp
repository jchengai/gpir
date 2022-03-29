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
#include <sstream>
#include "ad/map/route/ConnectingRouteType.hpp"
#include "ad/map/route/FullRoute.hpp"
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
 * \brief DataType ConnectingRoute
 *
 * A special route connecting two objects, A and B, in the map. The connection route
 * consists of two routes.
 * routeA is non empty, if the shorted path between A and B can be traveled by A in
 * forward direction.
 * routeB is non empty, if the shorted path between A and B can be traveled by B in
 * forward direction.
 * We have the following possibilities:
 * 1: There exists no connecting route (according to the search length/distance restrictions
 * ): both routeA and routeB are empty
 * 2: A and B are driving in same direction: one is able to reach the other with driving
 * forward, but not vice versa
 * 2.1: If A is in front of B: routeA is empty, routeB contains the route part from
 * B towards A
 * 2.2: If B is in front of A: routeB is empty, routeA contains the route part from
 * A towards B
 * 3. A and B driving in opposite direction: both are able to reach each other by driving
 * forward:
 * routeA contains the route part from A towards B and routeB the inverted route part
 * from B towards A.
 * 4. A and B can reach each other directly, but their routes merge at some point:
 * routeA contains the route part from A towards the merge point
 * routeB contains the route part from B towards the merge point
 */
struct ConnectingRoute
{
  /*!
   * \brief Smart pointer on ConnectingRoute
   */
  typedef std::shared_ptr<ConnectingRoute> Ptr;

  /*!
   * \brief Smart pointer on constant ConnectingRoute
   */
  typedef std::shared_ptr<ConnectingRoute const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ConnectingRoute() = default;

  /*!
   * \brief standard destructor
   */
  ~ConnectingRoute() = default;

  /*!
   * \brief standard copy constructor
   */
  ConnectingRoute(const ConnectingRoute &other) = default;

  /*!
   * \brief standard move constructor
   */
  ConnectingRoute(ConnectingRoute &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ConnectingRoute
   *
   * \returns Reference to this ConnectingRoute.
   */
  ConnectingRoute &operator=(const ConnectingRoute &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ConnectingRoute
   *
   * \returns Reference to this ConnectingRoute.
   */
  ConnectingRoute &operator=(ConnectingRoute &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ConnectingRoute
   *
   * \returns \c true if both ConnectingRoute are equal
   */
  bool operator==(const ConnectingRoute &other) const
  {
    return (type == other.type) && (routeA == other.routeA) && (routeB == other.routeB);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ConnectingRoute.
   *
   * \returns \c true if both ConnectingRoute are different
   */
  bool operator!=(const ConnectingRoute &other) const
  {
    return !operator==(other);
  }

  /*!
   * The type of connecting route.
   */
  ::ad::map::route::ConnectingRouteType type{::ad::map::route::ConnectingRouteType::Invalid};

  /*!
   * The route prefix of object A.
   * This part of the connected route is only used by object A to travel towards B.
   * If B is behind A, this part of the connecting route is empty.
   */
  ::ad::map::route::FullRoute routeA;

  /*!
   * The route prefix of object B.
   * This part of the connected route is only used by object B to travel towards A.
   * If A is behind B, this part of the connecting route is empty.
   */
  ::ad::map::route::FullRoute routeB;
};

} // namespace route
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_ROUTE_CONNECTINGROUTE
#define GEN_GUARD_AD_MAP_ROUTE_CONNECTINGROUTE
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
 * \param[in] _value ConnectingRoute value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ConnectingRoute const &_value)
{
  os << "ConnectingRoute(";
  os << "type:";
  os << _value.type;
  os << ",";
  os << "routeA:";
  os << _value.routeA;
  os << ",";
  os << "routeB:";
  os << _value.routeB;
  os << ")";
  return os;
}

} // namespace route
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ConnectingRoute
 */
inline std::string to_string(::ad::map::route::ConnectingRoute const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_ROUTE_CONNECTINGROUTE
