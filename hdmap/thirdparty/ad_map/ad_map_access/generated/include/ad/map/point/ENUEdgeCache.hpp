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

#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>

#include "ad/map/point/ENUEdge.hpp"
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
 * \brief DataType ENUEdgeCache
 */
struct ENUEdgeCache {
  /*!
   * \brief Smart pointer on ENUEdgeCache
   */
  typedef std::shared_ptr<ENUEdgeCache> Ptr;

  /*!
   * \brief Smart pointer on constant ENUEdgeCache
   */
  typedef std::shared_ptr<ENUEdgeCache const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ENUEdgeCache() = default;

  /*!
   * \brief standard destructor
   */
  ~ENUEdgeCache() = default;

  /*!
   * \brief standard copy constructor
   */
  ENUEdgeCache(const ENUEdgeCache &other) = default;

  /*!
   * \brief standard move constructor
   */
  ENUEdgeCache(ENUEdgeCache &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ENUEdgeCache
   *
   * \returns Reference to this ENUEdgeCache.
   */
  ENUEdgeCache &operator=(const ENUEdgeCache &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ENUEdgeCache
   *
   * \returns Reference to this ENUEdgeCache.
   */
  ENUEdgeCache &operator=(ENUEdgeCache &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUEdgeCache
   *
   * \returns \c true if both ENUEdgeCache are equal
   */
  bool operator==(const ENUEdgeCache &other) const {
    return (enuEdge == other.enuEdge) && (enuVersion == other.enuVersion);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUEdgeCache.
   *
   * \returns \c true if both ENUEdgeCache are different
   */
  bool operator!=(const ENUEdgeCache &other) const {
    return !operator==(other);
  }

  ::ad::map::point::ENUEdge enuEdge;
  uint64_t enuVersion{0};
};

}  // namespace point
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ENUEDGECACHE
#define GEN_GUARD_AD_MAP_POINT_ENUEDGECACHE
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
 * \param[in] _value ENUEdgeCache value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ENUEdgeCache const &_value) {
  os << "ENUEdgeCache(";
  os << "enuEdge:";
  os << _value.enuEdge;
  os << ",";
  os << "enuVersion:";
  os << _value.enuVersion;
  os << ")";
  return os;
}

}  // namespace point
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ENUEdgeCache
 */
inline std::string to_string(::ad::map::point::ENUEdgeCache const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_ENUEDGECACHE
