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
#include "ad/map/match/ENUObjectPosition.hpp"
#include "ad/map/match/MapMatchedObjectBoundingBox.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace match
 *
 * Map matching
 */
namespace match {

/*!
 * \brief DataType Object
 */
struct Object
{
  /*!
   * \brief Smart pointer on Object
   */
  typedef std::shared_ptr<Object> Ptr;

  /*!
   * \brief Smart pointer on constant Object
   */
  typedef std::shared_ptr<Object const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Object() = default;

  /*!
   * \brief standard destructor
   */
  ~Object() = default;

  /*!
   * \brief standard copy constructor
   */
  Object(const Object &other) = default;

  /*!
   * \brief standard move constructor
   */
  Object(Object &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Object
   *
   * \returns Reference to this Object.
   */
  Object &operator=(const Object &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Object
   *
   * \returns Reference to this Object.
   */
  Object &operator=(Object &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Object
   *
   * \returns \c true if both Object are equal
   */
  bool operator==(const Object &other) const
  {
    return (enuPosition == other.enuPosition) && (mapMatchedBoundingBox == other.mapMatchedBoundingBox);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Object.
   *
   * \returns \c true if both Object are different
   */
  bool operator!=(const Object &other) const
  {
    return !operator==(other);
  }

  /*!
   * Position of the object
   */
  ::ad::map::match::ENUObjectPosition enuPosition;
  ::ad::map::match::MapMatchedObjectBoundingBox mapMatchedBoundingBox;
};

} // namespace match
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_MATCH_OBJECT
#define GEN_GUARD_AD_MAP_MATCH_OBJECT
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace match
 *
 * Map matching
 */
namespace match {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value Object value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Object const &_value)
{
  os << "Object(";
  os << "enuPosition:";
  os << _value.enuPosition;
  os << ",";
  os << "mapMatchedBoundingBox:";
  os << _value.mapMatchedBoundingBox;
  os << ")";
  return os;
}

} // namespace match
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Object
 */
inline std::string to_string(::ad::map::match::Object const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_MATCH_OBJECT
