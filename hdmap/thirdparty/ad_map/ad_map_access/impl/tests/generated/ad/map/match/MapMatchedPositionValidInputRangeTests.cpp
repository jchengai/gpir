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

#include "ad/map/match/MapMatchedPositionValidInputRange.hpp"

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRange)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeLanePointTooSmall)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value below input range minimum
  ::ad::map::match::LanePoint invalidInitializedMember;
  ::ad::map::point::ParaPoint invalidInitializedMemberParaPoint;
  ::ad::physics::ParametricValue invalidInitializedMemberParaPointParametricOffset(
    0. - ::ad::physics::ParametricValue::cPrecisionValue);
  invalidInitializedMemberParaPoint.parametricOffset = invalidInitializedMemberParaPointParametricOffset;
  invalidInitializedMember.paraPoint = invalidInitializedMemberParaPoint;
  value.lanePoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeLanePointTooBig)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value above input range maximum
  ::ad::map::match::LanePoint invalidInitializedMember;
  ::ad::map::point::ParaPoint invalidInitializedMemberParaPoint;
  ::ad::physics::ParametricValue invalidInitializedMemberParaPointParametricOffset(1. * 1.1);
  invalidInitializedMemberParaPoint.parametricOffset = invalidInitializedMemberParaPointParametricOffset;
  invalidInitializedMember.paraPoint = invalidInitializedMemberParaPoint;
  value.lanePoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeTypeTooSmall)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value below input range minimum
  ::ad::map::match::MapMatchedPositionType invalidInitializedMember(
    static_cast<::ad::map::match::MapMatchedPositionType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeTypeTooBig)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value above input range maximum
  ::ad::map::match::MapMatchedPositionType invalidInitializedMember(
    static_cast<::ad::map::match::MapMatchedPositionType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeMatchedPointTooSmall)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(-6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.matchedPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeMatchedPointTooBig)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.matchedPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeProbabilityTooSmall)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value below input range minimum
  ::ad::physics::Probability invalidInitializedMember(0. - ::ad::physics::Probability::cPrecisionValue);
  value.probability = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeProbabilityTooBig)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value above input range maximum
  ::ad::physics::Probability invalidInitializedMember(1. * 1.1);
  value.probability = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeprobabilityDefault)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::physics::Probability valueDefault;
  value.probability = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeQueryPointTooSmall)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(-6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.queryPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeQueryPointTooBig)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFPoint invalidInitializedMember;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberX(6400000 * 1.1);
  invalidInitializedMember.x = invalidInitializedMemberX;
  value.queryPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeMatchedPointDistanceTooSmall)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.matchedPointDistance = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangeMatchedPointDistanceTooBig)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::map::match::LanePoint valueLanePoint;
  ::ad::map::point::ParaPoint valueLanePointParaPoint;
  ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
  valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
  ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
  valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
  valueLanePoint.paraPoint = valueLanePointParaPoint;
  ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  valueLanePoint.lateralT = valueLanePointLateralT;
  ::ad::physics::Distance valueLanePointLaneLength(-1e9);
  valueLanePoint.laneLength = valueLanePointLaneLength;
  ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
  valueLanePoint.laneWidth = valueLanePointLaneWidth;
  value.lanePoint = valueLanePoint;
  ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
  value.type = valueType;
  ::ad::map::point::ECEFPoint valueMatchedPoint;
  ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
  valueMatchedPoint.x = valueMatchedPointX;
  ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
  valueMatchedPoint.y = valueMatchedPointY;
  ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
  valueMatchedPoint.z = valueMatchedPointZ;
  value.matchedPoint = valueMatchedPoint;
  ::ad::physics::Probability valueProbability(0.);
  value.probability = valueProbability;
  ::ad::map::point::ECEFPoint valueQueryPoint;
  ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
  valueQueryPoint.x = valueQueryPointX;
  ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
  valueQueryPoint.y = valueQueryPointY;
  ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
  valueQueryPoint.z = valueQueryPointZ;
  value.queryPoint = valueQueryPoint;
  ::ad::physics::Distance valueMatchedPointDistance(-1e9);
  value.matchedPointDistance = valueMatchedPointDistance;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.matchedPointDistance = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(MapMatchedPositionValidInputRangeTests, testValidInputRangematchedPointDistanceDefault)
{
  ::ad::map::match::MapMatchedPosition value;
  ::ad::physics::Distance valueDefault;
  value.matchedPointDistance = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
