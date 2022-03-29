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
 * \brief DataType LandmarkType
 *
 * Type of landmark
 */
enum class LandmarkType : int32_t
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
   * Traffic sign (e.g. yield, stop).
   */
  TRAFFIC_SIGN = 2,

  /*!
   * Traffic light.
   */
  TRAFFIC_LIGHT = 3,

  /*!
   * Pole (e.g. street lamp).
   */
  POLE = 4,

  /*!
   * guide post
   */
  GUIDE_POST = 5,

  /*!
   * a tree
   */
  TREE = 6,

  /*!
   * street lamp
   */
  STREET_LAMP = 7,

  /*!
   * post box
   */
  POSTBOX = 8,

  /*!
   * a manhole on the street
   */
  MANHOLE = 9,

  /*!
   * electrical power box
   */
  POWERCABINET = 10,

  /*!
   * fire hydrant
   */
  FIRE_HYDRANT = 11,

  /*!
   * bollard (different/bigger to GUIDE_POST and POLE)
   */
  BOLLARD = 12,

  /*!
   * None of the above.
   */
  OTHER = 13
};

} // namespace landmark
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::landmark::LandmarkType to std::string helper.
 */
std::string toString(::ad::map::landmark::LandmarkType const e);

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
template <>::ad::map::landmark::LandmarkType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANDMARK_LANDMARKTYPE
#define GEN_GUARD_AD_MAP_LANDMARK_LANDMARKTYPE
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
 * \param[in] value LandmarkType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, LandmarkType const &value)
{
  return os << toString(value);
}

} // namespace landmark
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for LandmarkType
 */
inline std::string to_string(::ad::map::landmark::LandmarkType const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANDMARK_LANDMARKTYPE
