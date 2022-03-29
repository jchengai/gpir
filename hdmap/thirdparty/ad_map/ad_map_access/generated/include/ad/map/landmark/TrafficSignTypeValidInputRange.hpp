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

#include <cmath>
#include <limits>
#include "ad/map/landmark/TrafficSignType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given TrafficSignType is within valid input range
 *
 * \param[in] input the TrafficSignType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if TrafficSignType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::landmark::TrafficSignType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::landmark::TrafficSignType::INVALID)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY)
    || (input == ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::DANGER)
    || (input == ::ad::map::landmark::TrafficSignType::LANES_MERGING)
    || (input == ::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN)
    || (input == ::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN)
    || (input == ::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE)
    || (input == ::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS)
    || (input == ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER)
    || (input == ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING)
    || (input == ::ad::map::landmark::TrafficSignType::YIELD_TRAIN)
    || (input == ::ad::map::landmark::TrafficSignType::YIELD) || (input == ::ad::map::landmark::TrafficSignType::STOP)
    || (input == ::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN)
    || (input == ::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN)
    || (input == ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN)
    || (input == ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN)
    || (input == ::ad::map::landmark::TrafficSignType::ROUNDABOUT)
    || (input == ::ad::map::landmark::TrafficSignType::PASS_RIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::PASS_LEFT)
    || (input == ::ad::map::landmark::TrafficSignType::BYBICLE_PATH)
    || (input == ::ad::map::landmark::TrafficSignType::FOOTWALK)
    || (input == ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED)
    || (input == ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT)
    || (input == ::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT)
    || (input == ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR)
    || (input == ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN)
    || (input == ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END)
    || (input == ::ad::map::landmark::TrafficSignType::MAX_SPEED)
    || (input == ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN)
    || (input == ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END)
    || (input == ::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION)
    || (input == ::ad::map::landmark::TrafficSignType::PRIORITY_WAY)
    || (input == ::ad::map::landmark::TrafficSignType::CITY_BEGIN)
    || (input == ::ad::map::landmark::TrafficSignType::CITY_END)
    || (input == ::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN)
    || (input == ::ad::map::landmark::TrafficSignType::MOTORWAY_END)
    || (input == ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN)
    || (input == ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END)
    || (input == ::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO)
    || (input == ::ad::map::landmark::TrafficSignType::CUL_DE_SAC)
    || (input == ::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE)
    || (input == ::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN)
    || (input == ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN)
    || (input == ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL)
    || (input == ::ad::map::landmark::TrafficSignType::DESTINATION_BOARD)
    || (input == ::ad::map::landmark::TrafficSignType::FREE_TEXT)
    || (input == ::ad::map::landmark::TrafficSignType::UNKNOWN);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::landmark::TrafficSignType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
