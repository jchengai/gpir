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

#include "ad/map/lane/LaneIdValidInputRange.hpp"

TEST(LaneIdValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::map::lane::LaneId value(1);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneIdValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::map::lane::LaneId value(1 - 1);
  ASSERT_FALSE(withinValidInputRange(value));
}
