/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/*
 * Generated file
 */

#include <gtest/gtest.h>

#include <limits>

#include "ad/map/landmark/TrafficSignTypeValidInputRange.hpp"

TEST(TrafficSignTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::DANGER));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::LANES_MERGING));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::YIELD_TRAIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::YIELD));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::STOP));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ROUNDABOUT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::PASS_RIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::PASS_LEFT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::BYBICLE_PATH));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::FOOTWALK));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::MAX_SPEED));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::PRIORITY_WAY));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CITY_BEGIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CITY_END));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::MOTORWAY_END));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CUL_DE_SAC));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::DESTINATION_BOARD));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::FREE_TEXT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficSignType::UNKNOWN));
}

TEST(TrafficSignTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INVALID));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT));
  minValue = std::min(minValue,
                      static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN));
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE));
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED));
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED));
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DANGER));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::LANES_MERGING));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS));
  minValue = std::min(minValue,
                      static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::YIELD_TRAIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::YIELD));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::STOP));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ROUNDABOUT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PASS_RIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PASS_LEFT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::BYBICLE_PATH));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MAX_SPEED));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PRIORITY_WAY));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CITY_BEGIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CITY_END));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORWAY_END));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CUL_DE_SAC));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DESTINATION_BOARD));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FREE_TEXT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::UNKNOWN));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::landmark::TrafficSignType>(minValue - 1)));
}
