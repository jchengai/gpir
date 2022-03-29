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

#include "ad/map/landmark/TrafficSignType.hpp"
#include <stdexcept>

std::string toString(::ad::map::landmark::TrafficSignType const e)
{
  switch (e)
  {
    case ::ad::map::landmark::TrafficSignType::INVALID:
      return std::string("::ad::map::landmark::TrafficSignType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT:
      return std::string(
        "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE:
      return std::string(
        "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE:
      return std::string(
        "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME:
      return std::string(
        "::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED:
      return std::string(
        "::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN:
      return std::string(
        "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::DANGER:
      return std::string("::ad::map::landmark::TrafficSignType::DANGER"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::LANES_MERGING:
      return std::string("::ad::map::landmark::TrafficSignType::LANES_MERGING"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN:
      return std::string("::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN:
      return std::string("::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE:
      return std::string("::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS:
      return std::string("::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER:
      return std::string(
        "::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING:
      return std::string("::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::YIELD_TRAIN:
      return std::string("::ad::map::landmark::TrafficSignType::YIELD_TRAIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::YIELD:
      return std::string("::ad::map::landmark::TrafficSignType::YIELD"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::STOP:
      return std::string("::ad::map::landmark::TrafficSignType::STOP"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN:
      return std::string("::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN:
      return std::string("::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN:
      return std::string("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN:
      return std::string("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ROUNDABOUT:
      return std::string("::ad::map::landmark::TrafficSignType::ROUNDABOUT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::PASS_RIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::PASS_RIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::PASS_LEFT:
      return std::string("::ad::map::landmark::TrafficSignType::PASS_LEFT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::BYBICLE_PATH:
      return std::string("::ad::map::landmark::TrafficSignType::BYBICLE_PATH"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::FOOTWALK:
      return std::string("::ad::map::landmark::TrafficSignType::FOOTWALK"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED:
      return std::string("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT:
      return std::string("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN:
      return std::string("::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR:
      return std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN:
      return std::string("::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END:
      return std::string("::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::MAX_SPEED:
      return std::string("::ad::map::landmark::TrafficSignType::MAX_SPEED"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN:
      return std::string("::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END:
      return std::string("::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION:
      return std::string("::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::PRIORITY_WAY:
      return std::string("::ad::map::landmark::TrafficSignType::PRIORITY_WAY"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CITY_BEGIN:
      return std::string("::ad::map::landmark::TrafficSignType::CITY_BEGIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CITY_END:
      return std::string("::ad::map::landmark::TrafficSignType::CITY_END"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN:
      return std::string("::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::MOTORWAY_END:
      return std::string("::ad::map::landmark::TrafficSignType::MOTORWAY_END"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN:
      return std::string("::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END:
      return std::string("::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO:
      return std::string("::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CUL_DE_SAC:
      return std::string("::ad::map::landmark::TrafficSignType::CUL_DE_SAC"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE:
      return std::string("::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN:
      return std::string("::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN:
      return std::string("::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL:
      return std::string("::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::DESTINATION_BOARD:
      return std::string("::ad::map::landmark::TrafficSignType::DESTINATION_BOARD"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::FREE_TEXT:
      return std::string("::ad::map::landmark::TrafficSignType::FREE_TEXT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficSignType::UNKNOWN:
      return std::string("::ad::map::landmark::TrafficSignType::UNKNOWN"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::landmark::TrafficSignType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::landmark::TrafficSignType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::INVALID;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT;
  }
  if (str == std::string("SUPPLEMENT_ARROW_APPLIES_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT;
  }
  if (str == std::string("SUPPLEMENT_ARROW_APPLIES_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT;
  }
  if (str
      == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT;
  }
  if (str == std::string("SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN;
  }
  if (str == std::string("SUPPLEMENT_ARROW_APPLIES_UP_DOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN;
  }
  if (str
      == std::string(
           "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE;
  }
  if (str == std::string("SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE;
  }
  if (str == std::string(
               "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE;
  }
  if (str == std::string("SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE;
  }
  if (str
      == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME;
  }
  if (str == std::string("SUPPLEMENT_APPLIES_NEXT_N_KM_TIME")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS;
  }
  if (str == std::string("SUPPLEMENT_ENDS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED;
  }
  if (str == std::string("SUPPLEMENT_RESIDENTS_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED;
  }
  if (str == std::string("SUPPLEMENT_BICYCLE_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED;
  }
  if (str == std::string("SUPPLEMENT_MOPED_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED;
  }
  if (str == std::string("SUPPLEMENT_TRAM_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED;
  }
  if (str == std::string("SUPPLEMENT_FORESTAL_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED;
  }
  if (str == std::string(
               "::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED;
  }
  if (str == std::string("SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED;
  }
  if (str == std::string(
               "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN;
  }
  if (str == std::string("SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY;
  }
  if (str == std::string("SUPPLEMENT_RAILWAY_ONLY")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT;
  }
  if (str == std::string("SUPPLEMENT_APPLIES_FOR_WEIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::DANGER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DANGER;
  }
  if (str == std::string("DANGER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DANGER;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::LANES_MERGING")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::LANES_MERGING;
  }
  if (str == std::string("LANES_MERGING")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::LANES_MERGING;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN;
  }
  if (str == std::string("CAUTION_PEDESTRIAN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN;
  }
  if (str == std::string("CAUTION_CHILDREN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE;
  }
  if (str == std::string("CAUTION_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS;
  }
  if (str == std::string("CAUTION_ANIMALS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS;
  }
  if (str
      == std::string("::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER;
  }
  if (str == std::string("CAUTION_RAIL_CROSSING_WITH_BARRIER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING;
  }
  if (str == std::string("CAUTION_RAIL_CROSSING")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::YIELD_TRAIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::YIELD_TRAIN;
  }
  if (str == std::string("YIELD_TRAIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::YIELD_TRAIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::YIELD")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::YIELD;
  }
  if (str == std::string("YIELD")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::YIELD;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::STOP")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::STOP;
  }
  if (str == std::string("STOP")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::STOP;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN;
  }
  if (str == std::string("REQUIRED_RIGHT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN;
  }
  if (str == std::string("REQUIRED_LEFT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT;
  }
  if (str == std::string("REQUIRED_STRAIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN;
  }
  if (str == std::string("REQUIRED_STRAIGHT_OR_RIGHT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN;
  }
  if (str == std::string("REQUIRED_STRAIGHT_OR_LEFT_TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ROUNDABOUT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ROUNDABOUT;
  }
  if (str == std::string("ROUNDABOUT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ROUNDABOUT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::PASS_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PASS_RIGHT;
  }
  if (str == std::string("PASS_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PASS_RIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::PASS_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PASS_LEFT;
  }
  if (str == std::string("PASS_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PASS_LEFT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::BYBICLE_PATH")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::BYBICLE_PATH;
  }
  if (str == std::string("BYBICLE_PATH")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::BYBICLE_PATH;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::FOOTWALK")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK;
  }
  if (str == std::string("FOOTWALK")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED;
  }
  if (str == std::string("FOOTWALK_BICYCLE_SHARED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT;
  }
  if (str == std::string("FOOTWALK_BICYCLE_SEP_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT;
  }
  if (str == std::string("FOOTWALK_BICYCLE_SEP_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN;
  }
  if (str == std::string("PEDESTRIAN_AREA_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN;
  }
  if (str == std::string("ACCESS_FORBIDDEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS;
  }
  if (str == std::string("ACCESS_FORBIDDEN_TRUCKS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE;
  }
  if (str == std::string("ACCESS_FORBIDDEN_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES;
  }
  if (str == std::string("ACCESS_FORBIDDEN_MOTORVEHICLES")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT;
  }
  if (str == std::string("ACCESS_FORBIDDEN_WEIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH;
  }
  if (str == std::string("ACCESS_FORBIDDEN_WIDTH")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT;
  }
  if (str == std::string("ACCESS_FORBIDDEN_HEIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR;
  }
  if (str == std::string("ACCESS_FORBIDDEN_WRONG_DIR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN;
  }
  if (str == std::string("ENVIORNMENT_ZONE_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END;
  }
  if (str == std::string("ENVIORNMENT_ZONE_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::MAX_SPEED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MAX_SPEED;
  }
  if (str == std::string("MAX_SPEED")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MAX_SPEED;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN;
  }
  if (str == std::string("SPEED_ZONE_30_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END;
  }
  if (str == std::string("SPEED_ZONE_30_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION;
  }
  if (str == std::string("HAS_WAY_NEXT_INTERSECTION")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::PRIORITY_WAY")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PRIORITY_WAY;
  }
  if (str == std::string("PRIORITY_WAY")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::PRIORITY_WAY;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CITY_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CITY_BEGIN;
  }
  if (str == std::string("CITY_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CITY_BEGIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CITY_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CITY_END;
  }
  if (str == std::string("CITY_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CITY_END;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN;
  }
  if (str == std::string("MOTORWAY_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::MOTORWAY_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORWAY_END;
  }
  if (str == std::string("MOTORWAY_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORWAY_END;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN;
  }
  if (str == std::string("MOTORVEHICLE_BEGIN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END;
  }
  if (str == std::string("MOTORVEHICLE_END")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO;
  }
  if (str == std::string("INFO_MOTORWAY_INFO")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CUL_DE_SAC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CUL_DE_SAC;
  }
  if (str == std::string("CUL_DE_SAC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CUL_DE_SAC;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE;
  }
  if (str == std::string("CUL_DE_SAC_EXCEPT_PED_BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN;
  }
  if (str == std::string("INFO_NUMBER_OF_AUTOBAHN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN;
  }
  if (str == std::string("DIRECTION_TURN_TO_AUTOBAHN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL;
  }
  if (str == std::string("DIRECTION_TURN_TO_LOCAL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::DESTINATION_BOARD")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DESTINATION_BOARD;
  }
  if (str == std::string("DESTINATION_BOARD")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::DESTINATION_BOARD;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::FREE_TEXT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FREE_TEXT;
  }
  if (str == std::string("FREE_TEXT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::FREE_TEXT;
  }
  if (str == std::string("::ad::map::landmark::TrafficSignType::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficSignType::UNKNOWN;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
