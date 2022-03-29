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
 * @brief namespace match
 *
 * Map matching
 */
namespace match {

/*!
 * \brief DataType ObjectReferencePoints
 *
 * Defines reference points of an object
 */
enum class ObjectReferencePoints : int32_t
{
  /*!
   FrontLeft
   */
  FrontLeft = 0,
  /*!
   FrontRight
   */
  FrontRight = 1,
  /*!
   RearLeft
   */
  RearLeft = 2,
  /*!
   RearRight
   */
  RearRight = 3,
  /*!
   Center
   */
  Center = 4,
  /*!
   NumPoints
   */
  NumPoints = 5
};

} // namespace match
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::match::ObjectReferencePoints to std::string helper.
 */
std::string toString(::ad::map::match::ObjectReferencePoints const e);

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
template <>::ad::map::match::ObjectReferencePoints fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_MATCH_OBJECTREFERENCEPOINTS
#define GEN_GUARD_AD_MAP_MATCH_OBJECTREFERENCEPOINTS
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
 * \param[in] value ObjectReferencePoints value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ObjectReferencePoints const &value)
{
  return os << toString(value);
}

} // namespace match
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ObjectReferencePoints
 */
inline std::string to_string(::ad::map::match::ObjectReferencePoints const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_MATCH_OBJECTREFERENCEPOINTS
