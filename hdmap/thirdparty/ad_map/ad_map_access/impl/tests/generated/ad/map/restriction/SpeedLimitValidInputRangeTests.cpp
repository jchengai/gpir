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

#include "ad/map/restriction/SpeedLimitValidInputRange.hpp"

TEST(SpeedLimitValidInputRangeTests, testValidInputRange)
{
  ::ad::map::restriction::SpeedLimit value;
  ::ad::physics::Speed valueSpeedLimit(-100.);
  value.speedLimit = valueSpeedLimit;
  ::ad::physics::ParametricRange valueLanePiece;
  ::ad::physics::ParametricValue valueLanePieceMinimum(0.);
  valueLanePiece.minimum = valueLanePieceMinimum;
  ::ad::physics::ParametricValue valueLanePieceMaximum(0.);
  valueLanePiece.maximum = valueLanePieceMaximum;
  valueLanePiece.maximum = valueLanePiece.minimum;
  valueLanePiece.minimum = valueLanePiece.maximum;
  value.lanePiece = valueLanePiece;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedLimitValidInputRangeTests, testValidInputRangeSpeedLimitTooSmall)
{
  ::ad::map::restriction::SpeedLimit value;
  ::ad::physics::Speed valueSpeedLimit(-100.);
  value.speedLimit = valueSpeedLimit;
  ::ad::physics::ParametricRange valueLanePiece;
  ::ad::physics::ParametricValue valueLanePieceMinimum(0.);
  valueLanePiece.minimum = valueLanePieceMinimum;
  ::ad::physics::ParametricValue valueLanePieceMaximum(0.);
  valueLanePiece.maximum = valueLanePieceMaximum;
  valueLanePiece.maximum = valueLanePiece.minimum;
  valueLanePiece.minimum = valueLanePiece.maximum;
  value.lanePiece = valueLanePiece;

  // override member with data type value below input range minimum
  ::ad::physics::Speed invalidInitializedMember(-100. * 1.1);
  value.speedLimit = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedLimitValidInputRangeTests, testValidInputRangeSpeedLimitTooBig)
{
  ::ad::map::restriction::SpeedLimit value;
  ::ad::physics::Speed valueSpeedLimit(-100.);
  value.speedLimit = valueSpeedLimit;
  ::ad::physics::ParametricRange valueLanePiece;
  ::ad::physics::ParametricValue valueLanePieceMinimum(0.);
  valueLanePiece.minimum = valueLanePieceMinimum;
  ::ad::physics::ParametricValue valueLanePieceMaximum(0.);
  valueLanePiece.maximum = valueLanePieceMaximum;
  valueLanePiece.maximum = valueLanePiece.minimum;
  valueLanePiece.minimum = valueLanePiece.maximum;
  value.lanePiece = valueLanePiece;

  // override member with data type value above input range maximum
  ::ad::physics::Speed invalidInitializedMember(100. * 1.1);
  value.speedLimit = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedLimitValidInputRangeTests, testValidInputRangespeedLimitDefault)
{
  ::ad::map::restriction::SpeedLimit value;
  ::ad::physics::Speed valueDefault;
  value.speedLimit = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedLimitValidInputRangeTests, testValidInputRangeLanePieceTooSmall)
{
  ::ad::map::restriction::SpeedLimit value;
  ::ad::physics::Speed valueSpeedLimit(-100.);
  value.speedLimit = valueSpeedLimit;
  ::ad::physics::ParametricRange valueLanePiece;
  ::ad::physics::ParametricValue valueLanePieceMinimum(0.);
  valueLanePiece.minimum = valueLanePieceMinimum;
  ::ad::physics::ParametricValue valueLanePieceMaximum(0.);
  valueLanePiece.maximum = valueLanePieceMaximum;
  valueLanePiece.maximum = valueLanePiece.minimum;
  valueLanePiece.minimum = valueLanePiece.maximum;
  value.lanePiece = valueLanePiece;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricRange invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberMinimum(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.lanePiece = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedLimitValidInputRangeTests, testValidInputRangeLanePieceTooBig)
{
  ::ad::map::restriction::SpeedLimit value;
  ::ad::physics::Speed valueSpeedLimit(-100.);
  value.speedLimit = valueSpeedLimit;
  ::ad::physics::ParametricRange valueLanePiece;
  ::ad::physics::ParametricValue valueLanePieceMinimum(0.);
  valueLanePiece.minimum = valueLanePieceMinimum;
  ::ad::physics::ParametricValue valueLanePieceMaximum(0.);
  valueLanePiece.maximum = valueLanePieceMaximum;
  valueLanePiece.maximum = valueLanePiece.minimum;
  valueLanePiece.minimum = valueLanePiece.maximum;
  value.lanePiece = valueLanePiece;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricRange invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberMinimum(1. * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.lanePiece = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
