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
 * @brief namespace restriction
 *
 * Handling of traffic restrictions
 */
namespace restriction {

/*!
 * \brief DataType RoadUserType
 */
enum class RoadUserType : int32_t
{
  /*!
   INVALID
   */
  INVALID = 0,
  /*!
   UNKNOWN
   */
  UNKNOWN = 1,
  /*!
   CAR
   */
  CAR = 2,
  /*!
   BUS
   */
  BUS = 3,
  /*!
   TRUCK
   */
  TRUCK = 4,
  /*!
   PEDESTRIAN
   */
  PEDESTRIAN = 5,
  /*!
   MOTORBIKE
   */
  MOTORBIKE = 6,
  /*!
   BICYCLE
   */
  BICYCLE = 7,
  /*!
   CAR_ELECTRIC
   */
  CAR_ELECTRIC = 8,
  /*!
   CAR_HYBRID
   */
  CAR_HYBRID = 9,
  /*!
   CAR_PETROL
   */
  CAR_PETROL = 10,
  /*!
   CAR_DIESEL
   */
  CAR_DIESEL = 11
};

} // namespace restriction
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::restriction::RoadUserType to std::string helper.
 */
std::string toString(::ad::map::restriction::RoadUserType const e);

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
template <>::ad::map::restriction::RoadUserType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_RESTRICTION_ROADUSERTYPE
#define GEN_GUARD_AD_MAP_RESTRICTION_ROADUSERTYPE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace restriction
 *
 * Handling of traffic restrictions
 */
namespace restriction {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] value RoadUserType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, RoadUserType const &value)
{
  return os << toString(value);
}

} // namespace restriction
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for RoadUserType
 */
inline std::string to_string(::ad::map::restriction::RoadUserType const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_RESTRICTION_ROADUSERTYPE
