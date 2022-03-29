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

#include "ad/map/point/ParaPointValidInputRange.hpp"

TEST(ParaPointValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::ParaPoint value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueParametricOffset(0.);
  value.parametricOffset = valueParametricOffset;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParaPointValidInputRangeTests, testValidInputRangeParametricOffsetTooSmall)
{
  ::ad::map::point::ParaPoint value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueParametricOffset(0.);
  value.parametricOffset = valueParametricOffset;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricValue invalidInitializedMember(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.parametricOffset = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParaPointValidInputRangeTests, testValidInputRangeParametricOffsetTooBig)
{
  ::ad::map::point::ParaPoint value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueParametricOffset(0.);
  value.parametricOffset = valueParametricOffset;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricValue invalidInitializedMember(1. * 1.1);
  value.parametricOffset = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParaPointValidInputRangeTests, testValidInputRangeparametricOffsetDefault)
{
  ::ad::map::point::ParaPoint value;
  ::ad::physics::ParametricValue valueDefault;
  value.parametricOffset = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
