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

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

#include <gtest/gtest.h>
#include <limits>
#include "ad/map/landmark/TrafficSignType.hpp"

TEST(TrafficSignTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("INVALID"), ::ad::map::landmark::TrafficSignType::INVALID);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::INVALID"),
            ::ad::map::landmark::TrafficSignType::INVALID);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ARROW_APPLIES_LEFT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ARROW_APPLIES_RIGHT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ARROW_APPLIES_UP_DOWN"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_APPLIES_NEXT_N_KM_TIME"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ENDS"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_RESIDENTS_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_BICYCLE_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_MOPED_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED"),
    ::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_TRAM_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED"),
    ::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_FORESTAL_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_RAILWAY_ONLY"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY"),
    ::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SUPPLEMENT_APPLIES_FOR_WEIGHT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT"),
            ::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("DANGER"), ::ad::map::landmark::TrafficSignType::DANGER);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::DANGER"),
            ::ad::map::landmark::TrafficSignType::DANGER);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("LANES_MERGING"),
            ::ad::map::landmark::TrafficSignType::LANES_MERGING);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::LANES_MERGING"),
            ::ad::map::landmark::TrafficSignType::LANES_MERGING);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CAUTION_PEDESTRIAN"),
            ::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN"),
    ::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CAUTION_CHILDREN"),
            ::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN"),
            ::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CAUTION_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CAUTION_ANIMALS"),
            ::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS"),
            ::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CAUTION_RAIL_CROSSING_WITH_BARRIER"),
            ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER"),
            ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CAUTION_RAIL_CROSSING"),
            ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING"),
    ::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("YIELD_TRAIN"),
            ::ad::map::landmark::TrafficSignType::YIELD_TRAIN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::YIELD_TRAIN"),
            ::ad::map::landmark::TrafficSignType::YIELD_TRAIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("YIELD"), ::ad::map::landmark::TrafficSignType::YIELD);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::YIELD"),
            ::ad::map::landmark::TrafficSignType::YIELD);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("STOP"), ::ad::map::landmark::TrafficSignType::STOP);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::STOP"),
            ::ad::map::landmark::TrafficSignType::STOP);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("REQUIRED_RIGHT_TURN"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN"),
    ::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("REQUIRED_LEFT_TURN"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN"),
    ::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("REQUIRED_STRAIGHT"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("REQUIRED_STRAIGHT_OR_RIGHT_TURN"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("REQUIRED_STRAIGHT_OR_LEFT_TURN"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN"),
            ::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ROUNDABOUT"),
            ::ad::map::landmark::TrafficSignType::ROUNDABOUT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ROUNDABOUT"),
            ::ad::map::landmark::TrafficSignType::ROUNDABOUT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("PASS_RIGHT"),
            ::ad::map::landmark::TrafficSignType::PASS_RIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::PASS_RIGHT"),
            ::ad::map::landmark::TrafficSignType::PASS_RIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("PASS_LEFT"),
            ::ad::map::landmark::TrafficSignType::PASS_LEFT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::PASS_LEFT"),
            ::ad::map::landmark::TrafficSignType::PASS_LEFT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("BYBICLE_PATH"),
            ::ad::map::landmark::TrafficSignType::BYBICLE_PATH);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::BYBICLE_PATH"),
            ::ad::map::landmark::TrafficSignType::BYBICLE_PATH);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("FOOTWALK"),
            ::ad::map::landmark::TrafficSignType::FOOTWALK);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::FOOTWALK"),
            ::ad::map::landmark::TrafficSignType::FOOTWALK);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("FOOTWALK_BICYCLE_SHARED"),
            ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED"),
    ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("FOOTWALK_BICYCLE_SEP_RIGHT"),
            ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT"),
            ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("FOOTWALK_BICYCLE_SEP_LEFT"),
            ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT"),
    ::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("PEDESTRIAN_AREA_BEGIN"),
            ::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN"),
    ::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_TRUCKS"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS"),
    ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE"),
    ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_MOTORVEHICLES"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_WEIGHT"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT"),
    ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_WIDTH"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH"),
    ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_HEIGHT"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT"),
    ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ACCESS_FORBIDDEN_WRONG_DIR"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR"),
            ::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ENVIORNMENT_ZONE_BEGIN"),
            ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN"),
    ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("ENVIORNMENT_ZONE_END"),
            ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END"),
    ::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("MAX_SPEED"),
            ::ad::map::landmark::TrafficSignType::MAX_SPEED);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::MAX_SPEED"),
            ::ad::map::landmark::TrafficSignType::MAX_SPEED);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SPEED_ZONE_30_BEGIN"),
            ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN"),
    ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("SPEED_ZONE_30_END"),
            ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END"),
            ::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("HAS_WAY_NEXT_INTERSECTION"),
            ::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION"),
    ::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("PRIORITY_WAY"),
            ::ad::map::landmark::TrafficSignType::PRIORITY_WAY);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::PRIORITY_WAY"),
            ::ad::map::landmark::TrafficSignType::PRIORITY_WAY);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CITY_BEGIN"),
            ::ad::map::landmark::TrafficSignType::CITY_BEGIN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CITY_BEGIN"),
            ::ad::map::landmark::TrafficSignType::CITY_BEGIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CITY_END"),
            ::ad::map::landmark::TrafficSignType::CITY_END);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CITY_END"),
            ::ad::map::landmark::TrafficSignType::CITY_END);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("MOTORWAY_BEGIN"),
            ::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN"),
            ::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("MOTORWAY_END"),
            ::ad::map::landmark::TrafficSignType::MOTORWAY_END);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::MOTORWAY_END"),
            ::ad::map::landmark::TrafficSignType::MOTORWAY_END);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("MOTORVEHICLE_BEGIN"),
            ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN"),
    ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("MOTORVEHICLE_END"),
            ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END"),
            ::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("INFO_MOTORWAY_INFO"),
            ::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO"),
    ::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CUL_DE_SAC"),
            ::ad::map::landmark::TrafficSignType::CUL_DE_SAC);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::CUL_DE_SAC"),
            ::ad::map::landmark::TrafficSignType::CUL_DE_SAC);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("CUL_DE_SAC_EXCEPT_PED_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE"),
            ::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("INFO_NUMBER_OF_AUTOBAHN"),
            ::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN"),
    ::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("DIRECTION_TURN_TO_AUTOBAHN"),
            ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>(
              "::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN"),
            ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("DIRECTION_TURN_TO_LOCAL"),
            ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL"),
    ::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("DESTINATION_BOARD"),
            ::ad::map::landmark::TrafficSignType::DESTINATION_BOARD);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::DESTINATION_BOARD"),
            ::ad::map::landmark::TrafficSignType::DESTINATION_BOARD);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("FREE_TEXT"),
            ::ad::map::landmark::TrafficSignType::FREE_TEXT);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::FREE_TEXT"),
            ::ad::map::landmark::TrafficSignType::FREE_TEXT);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("UNKNOWN"), ::ad::map::landmark::TrafficSignType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficSignType>("::ad::map::landmark::TrafficSignType::UNKNOWN"),
            ::ad::map::landmark::TrafficSignType::UNKNOWN);

  EXPECT_ANY_THROW({ fromString<::ad::map::landmark::TrafficSignType>("NOT A VALID ENUM LITERAL"); });
}

TEST(TrafficSignTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::INVALID), "::ad::map::landmark::TrafficSignType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INVALID));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_RIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT");
  minValue = std::min(minValue,
                      static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT));
  maxValue = std::max(maxValue,
                      static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE");
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE));
  maxValue = std::max(
    maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_LEFT_RIGHT_BICYCLE));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE");
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE));
  maxValue = std::max(
    maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ARROW_APPLIES_UP_DOWN_BICYCLE));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_NEXT_N_KM_TIME));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENDS));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RESIDENTS_ALLOWED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_BICYCLE_ALLOWED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_MOPED_ALLOWED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_TRAM_ALLOWED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_FORESTAL_ALLOWED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED");
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED));
  maxValue = std::max(
    maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_CONSTRUCTION_VEHICLE_ALLOWED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN");
  minValue = std::min(
    minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN));
  maxValue = std::max(
    maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_ENVIRONMENT_ZONE_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_RAILWAY_ONLY));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT),
            "::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SUPPLEMENT_APPLIES_FOR_WEIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::DANGER), "::ad::map::landmark::TrafficSignType::DANGER");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DANGER));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DANGER));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::LANES_MERGING),
            "::ad::map::landmark::TrafficSignType::LANES_MERGING");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::LANES_MERGING));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::LANES_MERGING));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN),
            "::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_PEDESTRIAN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN),
            "::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_CHILDREN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE),
            "::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_BICYCLE));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS),
            "::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_ANIMALS));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER),
            "::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER");
  minValue = std::min(minValue,
                      static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER));
  maxValue = std::max(maxValue,
                      static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING_WITH_BARRIER));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING),
            "::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CAUTION_RAIL_CROSSING));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::YIELD_TRAIN),
            "::ad::map::landmark::TrafficSignType::YIELD_TRAIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::YIELD_TRAIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::YIELD_TRAIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::YIELD), "::ad::map::landmark::TrafficSignType::YIELD");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::YIELD));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::YIELD));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::STOP), "::ad::map::landmark::TrafficSignType::STOP");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::STOP));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::STOP));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN),
            "::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_RIGHT_TURN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN),
            "::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_LEFT_TURN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT),
            "::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN),
            "::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_RIGHT_TURN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN),
            "::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::REQUIRED_STRAIGHT_OR_LEFT_TURN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ROUNDABOUT),
            "::ad::map::landmark::TrafficSignType::ROUNDABOUT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ROUNDABOUT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ROUNDABOUT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::PASS_RIGHT),
            "::ad::map::landmark::TrafficSignType::PASS_RIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PASS_RIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PASS_RIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::PASS_LEFT),
            "::ad::map::landmark::TrafficSignType::PASS_LEFT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PASS_LEFT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PASS_LEFT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::BYBICLE_PATH),
            "::ad::map::landmark::TrafficSignType::BYBICLE_PATH");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::BYBICLE_PATH));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::BYBICLE_PATH));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::FOOTWALK), "::ad::map::landmark::TrafficSignType::FOOTWALK");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED),
            "::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SHARED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT),
            "::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_RIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT),
            "::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FOOTWALK_BICYCLE_SEP_LEFT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN),
            "::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PEDESTRIAN_AREA_BEGIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_TRUCKS));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_BICYCLE));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_MOTORVEHICLES));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WEIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WIDTH));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_HEIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR),
            "::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ACCESS_FORBIDDEN_WRONG_DIR));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN),
            "::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_BEGIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END),
            "::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::ENVIORNMENT_ZONE_END));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::MAX_SPEED),
            "::ad::map::landmark::TrafficSignType::MAX_SPEED");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MAX_SPEED));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MAX_SPEED));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN),
            "::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_BEGIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END),
            "::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::SPEED_ZONE_30_END));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION),
            "::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::HAS_WAY_NEXT_INTERSECTION));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::PRIORITY_WAY),
            "::ad::map::landmark::TrafficSignType::PRIORITY_WAY");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PRIORITY_WAY));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::PRIORITY_WAY));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CITY_BEGIN),
            "::ad::map::landmark::TrafficSignType::CITY_BEGIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CITY_BEGIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CITY_BEGIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CITY_END), "::ad::map::landmark::TrafficSignType::CITY_END");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CITY_END));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CITY_END));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN),
            "::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORWAY_BEGIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::MOTORWAY_END),
            "::ad::map::landmark::TrafficSignType::MOTORWAY_END");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORWAY_END));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORWAY_END));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN),
            "::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_BEGIN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END),
            "::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::MOTORVEHICLE_END));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO),
            "::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INFO_MOTORWAY_INFO));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CUL_DE_SAC),
            "::ad::map::landmark::TrafficSignType::CUL_DE_SAC");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CUL_DE_SAC));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CUL_DE_SAC));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE),
            "::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::CUL_DE_SAC_EXCEPT_PED_BICYCLE));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN),
            "::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::INFO_NUMBER_OF_AUTOBAHN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN),
            "::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_AUTOBAHN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL),
            "::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DIRECTION_TURN_TO_LOCAL));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::DESTINATION_BOARD),
            "::ad::map::landmark::TrafficSignType::DESTINATION_BOARD");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DESTINATION_BOARD));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::DESTINATION_BOARD));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::FREE_TEXT),
            "::ad::map::landmark::TrafficSignType::FREE_TEXT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FREE_TEXT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::FREE_TEXT));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficSignType::UNKNOWN), "::ad::map::landmark::TrafficSignType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficSignType::UNKNOWN));

  ASSERT_EQ(toString(static_cast<::ad::map::landmark::TrafficSignType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::landmark::TrafficSignType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(TrafficSignTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::landmark::TrafficSignType value(::ad::map::landmark::TrafficSignType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
