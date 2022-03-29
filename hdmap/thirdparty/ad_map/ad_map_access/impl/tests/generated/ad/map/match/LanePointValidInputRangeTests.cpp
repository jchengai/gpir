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

#include "ad/map/match/LanePointValidInputRange.hpp"

TEST(LanePointValidInputRangeTests, testValidInputRange)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangeParaPointTooSmall)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;

  // override member with data type value below input range minimum
  ::ad::map::point::ParaPoint invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberParametricOffset(
    0. - ::ad::physics::ParametricValue::cPrecisionValue);
  invalidInitializedMember.parametricOffset = invalidInitializedMemberParametricOffset;
  value.paraPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangeParaPointTooBig)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;

  // override member with data type value above input range maximum
  ::ad::map::point::ParaPoint invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberParametricOffset(1. * 1.1);
  invalidInitializedMember.parametricOffset = invalidInitializedMemberParametricOffset;
  value.paraPoint = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangeLaneLengthTooSmall)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.laneLength = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangeLaneLengthTooBig)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.laneLength = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangelaneLengthDefault)
{
  ::ad::map::match::LanePoint value;
  ::ad::physics::Distance valueDefault;
  value.laneLength = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangeLaneWidthTooSmall)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.laneWidth = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangeLaneWidthTooBig)
{
  ::ad::map::match::LanePoint value;
  ::ad::map::point::ParaPoint valueParaPoint;
  ::ad::map::lane::LaneId valueParaPointLaneId(1);
  valueParaPoint.laneId = valueParaPointLaneId;
  ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
  valueParaPoint.parametricOffset = valueParaPointParametricOffset;
  value.paraPoint = valueParaPoint;
  ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
  value.lateralT = valueLateralT;
  ::ad::physics::Distance valueLaneLength(-1e9);
  value.laneLength = valueLaneLength;
  ::ad::physics::Distance valueLaneWidth(-1e9);
  value.laneWidth = valueLaneWidth;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.laneWidth = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LanePointValidInputRangeTests, testValidInputRangelaneWidthDefault)
{
  ::ad::map::match::LanePoint value;
  ::ad::physics::Distance valueDefault;
  value.laneWidth = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
