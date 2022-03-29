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

#include "ad/map/restriction/SpeedLimitListValidInputRange.hpp"

TEST(SpeedLimitListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::restriction::SpeedLimitList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedLimitListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::restriction::SpeedLimitList value;
  ::ad::map::restriction::SpeedLimit element;
  ::ad::physics::Speed elementSpeedLimit(-100.);
  element.speedLimit = elementSpeedLimit;
  ::ad::physics::ParametricRange elementLanePiece;
  ::ad::physics::ParametricValue elementLanePieceMinimum(0.);
  elementLanePiece.minimum = elementLanePieceMinimum;
  ::ad::physics::ParametricValue elementLanePieceMaximum(0.);
  elementLanePiece.maximum = elementLanePieceMaximum;
  elementLanePiece.maximum = elementLanePiece.minimum;
  elementLanePiece.minimum = elementLanePiece.maximum;
  element.lanePiece = elementLanePiece;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedLimitListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::restriction::SpeedLimitList value;
  ::ad::map::restriction::SpeedLimit element;
  ::ad::physics::Speed elementSpeedLimit(-100. * 1.1);
  element.speedLimit = elementSpeedLimit;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
