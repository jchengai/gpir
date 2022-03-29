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
 * \brief DataType ContactLocation
 *
 * Specification of the location of the contact of an object.
 */
enum class ContactLocation : int32_t
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
   * Another object is on Left from this object.
   */
  LEFT = 2,

  /*!
   * Another object is on Right from this object.
   */
  RIGHT = 3,

  /*!
   * Another object is an Successor of this object.
   */
  SUCCESSOR = 4,

  /*!
   * Another object is an Predecessor of this object.
   */
  PREDECESSOR = 5,

  /*!
   * Another object Overlaps this object.
   */
  OVERLAP = 6
};

} // namespace lane
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::lane::ContactLocation to std::string helper.
 */
std::string toString(::ad::map::lane::ContactLocation const e);

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
template <>::ad::map::lane::ContactLocation fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_CONTACTLOCATION
#define GEN_GUARD_AD_MAP_LANE_CONTACTLOCATION
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
 * \param[in] value ContactLocation value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ContactLocation const &value)
{
  return os << toString(value);
}

} // namespace lane
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ContactLocation
 */
inline std::string to_string(::ad::map::lane::ContactLocation const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANE_CONTACTLOCATION
