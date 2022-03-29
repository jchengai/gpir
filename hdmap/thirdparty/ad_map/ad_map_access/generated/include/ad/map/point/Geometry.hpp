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

#include "ad/map/point/ECEFEdge.hpp"
#include "ad/map/point/ENUEdgeCache.hpp"
#include "ad/physics/Distance.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace point
 *
 * Handling geographic positions in different coordinate systems
 */
namespace point {

/*!
 * \brief DataType Geometry
 */
struct Geometry {
  /*!
   * \brief Smart pointer on Geometry
   */
  typedef std::shared_ptr<Geometry> Ptr;

  /*!
   * \brief Smart pointer on constant Geometry
   */
  typedef std::shared_ptr<Geometry const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Geometry() = default;

  /*!
   * \brief standard destructor
   */
  ~Geometry() = default;

  /*!
   * \brief standard copy constructor
   */
  Geometry(const Geometry &other) = default;

  /*!
   * \brief standard move constructor
   */
  Geometry(Geometry &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Geometry
   *
   * \returns Reference to this Geometry.
   */
  Geometry &operator=(const Geometry &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Geometry
   *
   * \returns Reference to this Geometry.
   */
  Geometry &operator=(Geometry &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Geometry
   *
   * \returns \c true if both Geometry are equal
   */
  bool operator==(const Geometry &other) const {
    return (isValid == other.isValid) && (isClosed == other.isClosed) &&
           (ecefEdge == other.ecefEdge) && (length == other.length) &&
           (private_enuEdgeCache == other.private_enuEdgeCache);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Geometry.
   *
   * \returns \c true if both Geometry are different
   */
  bool operator!=(const Geometry &other) const { return !operator==(other); }

  bool isValid{false};
  bool isClosed{false};
  ::ad::map::point::ECEFEdge ecefEdge;
  ::ad::physics::Distance length;

  /*!
   * Private member storing the ecefEdge as enuEdge.
   * Access this member only through respective interface functions to ensure
   * the cache is updated when required.
   */
  ::ad::map::point::ENUEdgeCache private_enuEdgeCache;
};

}  // namespace point
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_GEOMETRY
#define GEN_GUARD_AD_MAP_POINT_GEOMETRY
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace point
 *
 * Handling geographic positions in different coordinate systems
 */
namespace point {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value Geometry value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Geometry const &_value) {
  os << "Geometry(";
  os << "isValid:";
  os << _value.isValid;
  os << ",";
  os << "isClosed:";
  os << _value.isClosed;
  os << ",";
  os << "ecefEdge:";
  os << _value.ecefEdge;
  os << ",";
  os << "length:";
  os << _value.length;
  os << ",";
  os << "private_enuEdgeCache:";
  os << _value.private_enuEdgeCache;
  os << ")";
  return os;
}

}  // namespace point
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Geometry
 */
inline std::string to_string(::ad::map::point::Geometry const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_GEOMETRY
