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

#include "ad/map/route/RouteParaPointValidInputRange.hpp"

TEST(RouteParaPointValidInputRangeTests, testValidInputRange)
{
  ::ad::map::route::RouteParaPoint value;
  ::ad::map::route::RoutePlanningCounter valueRoutePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
  value.routePlanningCounter = valueRoutePlanningCounter;
  ::ad::map::route::SegmentCounter valueSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
  value.segmentCountFromDestination = valueSegmentCountFromDestination;
  ::ad::physics::ParametricValue valueParametricOffset(0.);
  value.parametricOffset = valueParametricOffset;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RouteParaPointValidInputRangeTests, testValidInputRangeParametricOffsetTooSmall)
{
  ::ad::map::route::RouteParaPoint value;
  ::ad::map::route::RoutePlanningCounter valueRoutePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
  value.routePlanningCounter = valueRoutePlanningCounter;
  ::ad::map::route::SegmentCounter valueSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
  value.segmentCountFromDestination = valueSegmentCountFromDestination;
  ::ad::physics::ParametricValue valueParametricOffset(0.);
  value.parametricOffset = valueParametricOffset;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricValue invalidInitializedMember(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.parametricOffset = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(RouteParaPointValidInputRangeTests, testValidInputRangeParametricOffsetTooBig)
{
  ::ad::map::route::RouteParaPoint value;
  ::ad::map::route::RoutePlanningCounter valueRoutePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
  value.routePlanningCounter = valueRoutePlanningCounter;
  ::ad::map::route::SegmentCounter valueSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
  value.segmentCountFromDestination = valueSegmentCountFromDestination;
  ::ad::physics::ParametricValue valueParametricOffset(0.);
  value.parametricOffset = valueParametricOffset;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricValue invalidInitializedMember(1. * 1.1);
  value.parametricOffset = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(RouteParaPointValidInputRangeTests, testValidInputRangeparametricOffsetDefault)
{
  ::ad::map::route::RouteParaPoint value;
  ::ad::physics::ParametricValue valueDefault;
  value.parametricOffset = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
