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
 * \brief DataType TrafficSignType
 *
 * Type of traffic sign
 */
enum class TrafficSignType : int32_t
{
  /*!
   INVALID
   */
  INVALID = 0,
  /*!
   SUPPLEMENT_ARROW_APPLIES_LEFT
   */
  SUPPLEMENT_ARROW_APPLIES_LEFT = 1,
  /*!
   SUPPLEMENT_ARROW_APPLIES_RIGHT
   */
  SUPPLEMENT_ARROW_APPLIES_RIGHT = 2,
  /*!
   SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT
   */
  SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT = 3,
  /*!
   SUPPLEMENT_ARROW_APPLIES_UP_DOWN
   */
  SUPPLEMENT_ARROW_APPLIES_UP_DOWN = 4,
  /*!
   SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE
   */
  SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE = 5,
  /*!
   SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE
   */
  SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE = 6,
  /*!
   SUPPLEMENT_APPLIES_NEXT_N_KM_TIME
   */
  SUPPLEMENT_APPLIES_NEXT_N_KM_TIME = 7,
  /*!
   SUPPLEMENT_ENDS
   */
  SUPPLEMENT_ENDS = 8,
  /*!
   SUPPLEMENT_RESIDENTS_ALLOWED
   */
  SUPPLEMENT_RESIDENTS_ALLOWED = 9,
  /*!
   SUPPLEMENT_BICYCLE_ALLOWED
   */
  SUPPLEMENT_BICYCLE_ALLOWED = 10,
  /*!
   SUPPLEMENT_MOPED_ALLOWED
   */
  SUPPLEMENT_MOPED_ALLOWED = 11,
  /*!
   SUPPLEMENT_TRAM_ALLOWED
   */
  SUPPLEMENT_TRAM_ALLOWED = 12,
  /*!
   SUPPLEMENT_FORESTAL_ALLOWED
   */
  SUPPLEMENT_FORESTAL_ALLOWED = 13,
  /*!
   SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED
   */
  SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED = 14,
  /*!
   SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN
   */
  SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN = 15,
  /*!
   SUPPLEMENT_RAILWAY_ONLY
   */
  SUPPLEMENT_RAILWAY_ONLY = 16,
  /*!
   SUPPLEMENT_APPLIES_FOR_WEIGHT
   */
  SUPPLEMENT_APPLIES_FOR_WEIGHT = 17,
  /*!
   DANGER
   */
  DANGER = 18,
  /*!
   LANES_MERGING
   */
  LANES_MERGING = 19,
  /*!
   CAUTION_PEDESTRIAN
   */
  CAUTION_PEDESTRIAN = 20,
  /*!
   CAUTION_CHILDREN
   */
  CAUTION_CHILDREN = 21,
  /*!
   CAUTION_BICYCLE
   */
  CAUTION_BICYCLE = 22,
  /*!
   CAUTION_ANIMALS
   */
  CAUTION_ANIMALS = 23,
  /*!
   CAUTION_RAIL_CROSSING_WITH_BARRIER
   */
  CAUTION_RAIL_CROSSING_WITH_BARRIER = 24,
  /*!
   CAUTION_RAIL_CROSSING
   */
  CAUTION_RAIL_CROSSING = 25,
  /*!
   YIELD_TRAIN
   */
  YIELD_TRAIN = 26,
  /*!
   YIELD
   */
  YIELD = 27,
  /*!
   STOP
   */
  STOP = 28,
  /*!
   REQUIRED_RIGHT_TURN
   */
  REQUIRED_RIGHT_TURN = 29,
  /*!
   REQUIRED_LEFT_TURN
   */
  REQUIRED_LEFT_TURN = 30,
  /*!
   REQUIRED_STRAIGHT
   */
  REQUIRED_STRAIGHT = 31,
  /*!
   REQUIRED_STRAIGHT_OR_RIGHT_TURN
   */
  REQUIRED_STRAIGHT_OR_RIGHT_TURN = 32,
  /*!
   REQUIRED_STRAIGHT_OR_LEFT_TURN
   */
  REQUIRED_STRAIGHT_OR_LEFT_TURN = 33,
  /*!
   ROUNDABOUT
   */
  ROUNDABOUT = 34,
  /*!
   PASS_RIGHT
   */
  PASS_RIGHT = 35,
  /*!
   PASS_LEFT
   */
  PASS_LEFT = 36,
  /*!
   BYBICLE_PATH
   */
  BYBICLE_PATH = 37,
  /*!
   FOOTWALK
   */
  FOOTWALK = 38,
  /*!
   FOOTWALK_BICYCLE_SHARED
   */
  FOOTWALK_BICYCLE_SHARED = 39,
  /*!
   FOOTWALK_BICYCLE_SEP_RIGHT
   */
  FOOTWALK_BICYCLE_SEP_RIGHT = 40,
  /*!
   FOOTWALK_BICYCLE_SEP_LEFT
   */
  FOOTWALK_BICYCLE_SEP_LEFT = 41,
  /*!
   PEDESTRIAN_AREA_BEGIN
   */
  PEDESTRIAN_AREA_BEGIN = 42,
  /*!
   ACCESS_FORBIDDEN
   */
  ACCESS_FORBIDDEN = 43,
  /*!
   ACCESS_FORBIDDEN_TRUCKS
   */
  ACCESS_FORBIDDEN_TRUCKS = 44,
  /*!
   ACCESS_FORBIDDEN_BICYCLE
   */
  ACCESS_FORBIDDEN_BICYCLE = 45,
  /*!
   ACCESS_FORBIDDEN_MOTORVEHICLES
   */
  ACCESS_FORBIDDEN_MOTORVEHICLES = 46,
  /*!
   ACCESS_FORBIDDEN_WEIGHT
   */
  ACCESS_FORBIDDEN_WEIGHT = 47,
  /*!
   ACCESS_FORBIDDEN_WIDTH
   */
  ACCESS_FORBIDDEN_WIDTH = 48,
  /*!
   ACCESS_FORBIDDEN_HEIGHT
   */
  ACCESS_FORBIDDEN_HEIGHT = 49,
  /*!
   ACCESS_FORBIDDEN_WRONG_DIR
   */
  ACCESS_FORBIDDEN_WRONG_DIR = 50,
  /*!
   ENVIORNMENT_ZONE_BEGIN
   */
  ENVIORNMENT_ZONE_BEGIN = 51,
  /*!
   ENVIORNMENT_ZONE_END
   */
  ENVIORNMENT_ZONE_END = 52,
  /*!
   MAX_SPEED
   */
  MAX_SPEED = 53,
  /*!
   SPEED_ZONE_30_BEGIN
   */
  SPEED_ZONE_30_BEGIN = 54,
  /*!
   SPEED_ZONE_30_END
   */
  SPEED_ZONE_30_END = 55,
  /*!
   HAS_WAY_NEXT_INTERSECTION
   */
  HAS_WAY_NEXT_INTERSECTION = 56,
  /*!
   PRIORITY_WAY
   */
  PRIORITY_WAY = 57,
  /*!
   CITY_BEGIN
   */
  CITY_BEGIN = 58,
  /*!
   CITY_END
   */
  CITY_END = 59,
  /*!
   MOTORWAY_BEGIN
   */
  MOTORWAY_BEGIN = 60,
  /*!
   MOTORWAY_END
   */
  MOTORWAY_END = 61,
  /*!
   MOTORVEHICLE_BEGIN
   */
  MOTORVEHICLE_BEGIN = 62,
  /*!
   MOTORVEHICLE_END
   */
  MOTORVEHICLE_END = 63,
  /*!
   INFO_MOTORWAY_INFO
   */
  INFO_MOTORWAY_INFO = 64,
  /*!
   CUL_DE_SAC
   */
  CUL_DE_SAC = 65,
  /*!
   CUL_DE_SAC_EXCEPT_PED_BICYCLE
   */
  CUL_DE_SAC_EXCEPT_PED_BICYCLE = 66,
  /*!
   INFO_NUMBER_OF_AUTOBAHN
   */
  INFO_NUMBER_OF_AUTOBAHN = 67,
  /*!
   DIRECTION_TURN_TO_AUTOBAHN
   */
  DIRECTION_TURN_TO_AUTOBAHN = 68,
  /*!
   DIRECTION_TURN_TO_LOCAL
   */
  DIRECTION_TURN_TO_LOCAL = 69,
  /*!
   DESTINATION_BOARD
   */
  DESTINATION_BOARD = 70,
  /*!
   FREE_TEXT
   */
  FREE_TEXT = 71,
  /*!
   UNKNOWN
   */
  UNKNOWN = 72
};

} // namespace landmark
} // namespace map
} // namespace ad
/*!
 * \brief Conversion of ::ad::map::landmark::TrafficSignType to std::string helper.
 */
std::string toString(::ad::map::landmark::TrafficSignType const e);

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
template <>::ad::map::landmark::TrafficSignType fromString(std::string const &str);

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANDMARK_TRAFFICSIGNTYPE
#define GEN_GUARD_AD_MAP_LANDMARK_TRAFFICSIGNTYPE
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
 * \param[in] value TrafficSignType value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, TrafficSignType const &value)
{
  return os << toString(value);
}

} // namespace landmark
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for TrafficSignType
 */
inline std::string to_string(::ad::map::landmark::TrafficSignType const &value)
{
  return ::toString(value);
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANDMARK_TRAFFICSIGNTYPE
